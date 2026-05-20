/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *                Copyright (C) 2023.
 *
 * See the NOTICE file distributed with this work for additional information
 * regarding copyright ownership.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *             http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package Scenarios.Engine;

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.controller.ExecController;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.distl.AlwaysDisTLFormula;
import it.unicam.quasylab.jspear.distl.ConjunctionDisTLFormula;
import it.unicam.quasylab.jspear.distl.DisTLFormula;
import it.unicam.quasylab.jspear.distl.DoubleSemanticsVisitor;
import it.unicam.quasylab.jspear.distl.TargetDisTLFormula;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class StarkShieldApp {
    public int predictFutureSeconds = 1;
    public static final int auxilaryVarNums = 4;
    private static final double CRASH_DISTANCE_THRESHOLD = 0.0;
    private static final double STABILITY_DISTANCE_THRESHOLD = 0.1;
    private static final double MIN_ACCEPTABLE_ROBUSTNESS = 0.0;
    private static final double VEHICLE_LENGTH = 5.0;
    private static final double MIN_STABLE_FRONT_GAP = 15.0;
    private static final double MAX_STABLE_RELATIVE_SPEED = 2.0;
    private static final int EVOLUTION_SEQUENCE_SIZE = 10;
    //public int stepCount = 0;
    private List<Vehicle> finalVehicles;

    public double dt = 0;
    private HighwayEngine engine;
    private List<Vehicle> vehicles;
    public int STEPS_PER_SECOND;
    private DataState initialState;
    private ControlledSystem system;
    private EvolutionSequence sequence;
    private SampleSet<SystemState> dss;

    public StarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds) {
        this.engine = engine;
        this.dt = engine.dt;
        this.STEPS_PER_SECOND = engine.STEPS_PER_SECOND;
        this.predictFutureSeconds = predictFutureSeconds;

        this.vehicles = vehicles;
        initialState = this.getInitialState(vehicles);
        system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(this.getEnvironmentUpdates(rg, ds)), initialState);
        sequence = new EvolutionSequence(new SilentMonitor("Vehicle"), new DefaultRandomGenerator(), rg -> system, EVOLUTION_SEQUENCE_SIZE);
        printSummary();
    }

    private void printSummary() {
        dss = sequence.get(this.predictFutureSeconds * STEPS_PER_SECOND - 1);
       // SampleSet<SystemState> dss = sequence.get(1);
        dss.stream().limit(5).forEach(ss -> {
            System.out.println("Summary of the evolution sequence:");
            DataState ds = ss.getDataState();
            for (int i = 0; i < vehicles.size(); i++) {
                Vehicle v = stateToVehicle(ds, i);
                System.out.println(v);
            }
            System.out.println("Crashed:"+ ds.get(vehicles.size() * VarTable.values().length));
            System.out.println("Index of the car that is ahead of ego in the current lane:"+ ds.get(vehicles.size() * VarTable.values().length + 1));
            System.out.println("Index of the car that is ahead of ego in the left lane:"+ ds.get(vehicles.size() * VarTable.values().length + 2));
            System.out.println("Index of the car that is ahead of ego in the right lane:"+ ds.get(vehicles.size() * VarTable.values().length + 3));
            System.out.println("final vehicles");

        });

    }

    public boolean verifySafe(){
        int lastStep = this.predictFutureSeconds * STEPS_PER_SECOND - 1;
        DisTLFormula noCollision = new AlwaysDisTLFormula(
                new TargetDisTLFormula(this::resetCrashState, this::crashPenalty, CRASH_DISTANCE_THRESHOLD),
                0,
                lastStep
        );
        DisTLFormula stableAtLastStep = new AlwaysDisTLFormula(
                new TargetDisTLFormula(this::stabilizeEgoAgainstFrontVehicle, this::frontVehicleStabilityPenalty, STABILITY_DISTANCE_THRESHOLD),
                lastStep,
                lastStep
        );
        DisTLFormula shieldCondition = new ConjunctionDisTLFormula(noCollision, stableAtLastStep);
        double robustness = new DoubleSemanticsVisitor().eval(shieldCondition).eval(EVOLUTION_SEQUENCE_SIZE, 0, sequence);
        return robustness >= MIN_ACCEPTABLE_ROBUSTNESS;
    }

    private DataState resetCrashState(RandomGenerator rg, DataState state) {
        return state.apply(List.of(new DataStateUpdate(crashedIndex(), 0.0)));
    }

    private DataState stabilizeEgoAgainstFrontVehicle(RandomGenerator rg, DataState state) {
        int egoIndex = getEgoVehicleIndex(state);
        if (egoIndex < 0) {
            return state;
        }

        int frontIndex = getNearestFrontVehicleIndexAcrossAdjacentLanes(state, egoIndex);
        if (frontIndex < 0) {
            return state;
        }
        int egoOffset = vehicleOffset(egoIndex);
        int frontOffset = vehicleOffset(frontIndex);
        double frontX = state.get(frontOffset + VarTable.x.ordinal());
        double frontVx = state.get(frontOffset + VarTable.vx.ordinal());
        double frontSpeed = state.get(frontOffset + VarTable.speed.ordinal());
        return state.apply(List.of(
                new DataStateUpdate(egoOffset + VarTable.x.ordinal(), frontX - VEHICLE_LENGTH - MIN_STABLE_FRONT_GAP),
                new DataStateUpdate(egoOffset + VarTable.vx.ordinal(), frontVx),
                new DataStateUpdate(egoOffset + VarTable.speed.ordinal(), frontSpeed),
                new DataStateUpdate(egoOffset + VarTable.targetSpeed.ordinal(), frontSpeed)
        ));
    }

    private double crashPenalty(DataState state) {
        return state.get(crashedIndex()) > 0.0 ? 1.0 : 0.0;
    }

    private double frontVehicleStabilityPenalty(DataState state) {
        int egoIndex = getEgoVehicleIndex(state);
        if (egoIndex < 0) {
            return 0.0;
        }

        double totalPenalty = 0.0;
        int egoLane = (int) state.get(vehicleOffset(egoIndex) + VarTable.lane_index.ordinal());
        for (int lane = egoLane - 1; lane <= egoLane + 1; lane++) {
            int frontIndex = getFrontVehicleIndexInLane(state, egoIndex, lane);
            if (frontIndex >= 0) {
                totalPenalty += frontVehicleStabilityPenalty(state, egoIndex, frontIndex);
            }
        }
        return totalPenalty;
    }

    private double frontVehicleStabilityPenalty(DataState state, int egoIndex, int frontIndex) {
        int egoOffset = vehicleOffset(egoIndex);
        int frontOffset = vehicleOffset(frontIndex);
        double egoX = state.get(egoOffset + VarTable.x.ordinal());
        double frontX = state.get(frontOffset + VarTable.x.ordinal());
        double egoVx = state.get(egoOffset + VarTable.vx.ordinal());
        double frontVx = state.get(frontOffset + VarTable.vx.ordinal());
        double frontGap = frontX - egoX - VEHICLE_LENGTH;
        double closingSpeed = Math.max(0.0, egoVx - frontVx);
        double distanceViolation = Math.max(0.0, MIN_STABLE_FRONT_GAP - frontGap) / MIN_STABLE_FRONT_GAP;
        double speedViolation = Math.max(0.0, closingSpeed - MAX_STABLE_RELATIVE_SPEED) / MAX_STABLE_RELATIVE_SPEED;
        return Math.min(1.0, distanceViolation + speedViolation);
    }

    private int getEgoVehicleIndex(DataState state) {
        for (int i = 0; i < vehicles.size(); i++) {
            if (state.get(vehicleOffset(i) + VarTable.role.ordinal()) == 0.0) {
                return i;
            }
        }
        return -1;
    }

    private int getNearestFrontVehicleIndexAcrossAdjacentLanes(DataState state, int egoIndex) {
        if (egoIndex < 0) {
            return -1;
        }

        int egoOffset = vehicleOffset(egoIndex);
        int egoLane = (int) state.get(egoOffset + VarTable.lane_index.ordinal());
        double egoX = state.get(egoOffset + VarTable.x.ordinal());
        int nearestFrontIndex = -1;
        double nearestFrontX = Double.POSITIVE_INFINITY;
        for (int lane = egoLane - 1; lane <= egoLane + 1; lane++) {
            int candidateIndex = getFrontVehicleIndexInLane(state, egoIndex, lane);
            if (candidateIndex >= 0) {
                double candidateX = state.get(vehicleOffset(candidateIndex) + VarTable.x.ordinal());
                if (candidateX > egoX && candidateX < nearestFrontX) {
                    nearestFrontIndex = candidateIndex;
                    nearestFrontX = candidateX;
                }
            }
        }
        return nearestFrontIndex;
    }

    private int getFrontVehicleIndexInLane(DataState state, int egoIndex, int targetLane) {
        if (egoIndex < 0) {
            return -1;
        }

        int egoOffset = vehicleOffset(egoIndex);
        double egoX = state.get(egoOffset + VarTable.x.ordinal());
        int frontIndex = -1;
        double closestFrontX = Double.POSITIVE_INFINITY;
        for (int i = 0; i < vehicles.size(); i++) {
            if (i == egoIndex) {
                continue;
            }
            int offset = vehicleOffset(i);
            int lane = (int) state.get(offset + VarTable.lane_index.ordinal());
            double x = state.get(offset + VarTable.x.ordinal());
            if (lane == targetLane && x > egoX && x < closestFrontX) {
                frontIndex = i;
                closestFrontX = x;
            }
        }
        return frontIndex;
    }

    private int vehicleOffset(int vehicleIndex) {
        return vehicleIndex * VarTable.values().length;
    }

    private int crashedIndex() {
        return vehicles.size() * VarTable.values().length;
    }

    private DataState getInitialState(List<Vehicle> vehicles) {
        System.out.println("initial state fetched by stark:");
        for (Vehicle v : vehicles) {

            System.out.println(v);
        }

        Map<Integer, Double> values = new HashMap<>();
        for (int i = 0; i < vehicles.size(); i++) {
            Vehicle v = vehicles.get(i);
            int offSet = i * VarTable.values().length;
            values.put(offSet + VarTable.id.ordinal(),Double.valueOf(v.id));
            values.put(offSet + VarTable.politeness.ordinal(), v.politeness);
            values.put(offSet + VarTable.cooldownTimer.ordinal(), v.cooldownTimer);
            values.put(offSet + VarTable.target_lane_index.ordinal(), (double)v.getTargetLaneIndex());
            values.put(offSet + VarTable.lane_index.ordinal(), (double)v.getLaneIndex());
            values.put(offSet + VarTable.x.ordinal(), v.x);
            values.put(offSet + VarTable.y.ordinal(), v.y);
            values.put(offSet + VarTable.vx.ordinal(), v.vx);
            values.put(offSet + VarTable.vy.ordinal(), v.vy);
            values.put(offSet + VarTable.speed.ordinal(), v.speed);
            values.put(offSet + VarTable.heading.ordinal(), v.heading);
            values.put(offSet + VarTable.plannedAcceleration.ordinal(), v.plannedAcceleration);
            values.put(offSet + VarTable.plannedSteering.ordinal(), v.plannedSteering);
            values.put(offSet + VarTable.role.ordinal(), v.role.equals("EGO") ? 0.0 : 1.0);
            values.put(offSet + VarTable.targetSpeed.ordinal(), v.targetSpeed);
            if(v.role.equals("EGO")){
                System.out.println("starked ego intention targetspeed: " + v.targetSpeed + ", current speed: " + v.speed);
            }
        }
        //the last vars:
        //1. crashed
        values.put(vehicles.size() * VarTable.values().length, 0.0);
        //2. index of the car that is ahead of ego in the current lane
        values.put(vehicles.size() * VarTable.values().length + 1, -1.0);
        //3. index of the car that is ahead in the left lane
        values.put(vehicles.size() * VarTable.values().length + 2, -1.0);
        //4. index of the car that is ahead in the right lane
        values.put(vehicles.size() * VarTable.values().length + 3, -1.0);


//        return new DataState(vehicles.size() * VarTable.values().length + 1,
//                i -> values.getOrDefault(i, Double.NaN));
        return new DataState(values.size(),i -> values.getOrDefault(i, Double.NaN));
    }

    public Controller getController() {
        ControllerRegistry registry = new ControllerRegistry();
        Controller doNothing = Controller.doAction(
                (_rg, _ds) -> List.of(),
                registry.reference("doNothing"));
        registry.set("doNothing", doNothing);
        return new ExecController(registry.reference("doNothing"));
    }

    public List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        int numsVehicles = vehicles.size();
        List<Vehicle> localVehicles = new LinkedList<>();

        for (int i = 0; i < numsVehicles; i++) {
            Vehicle v = stateToVehicle(state, i);
            localVehicles.add(v);

        }


       //HighwayEngine sandboxEngine = new HighwayEngine();
        SandboxHighwayEngine sandboxEngine = new SandboxHighwayEngine();
        sandboxEngine.dt = this.dt;
        sandboxEngine.STEPS_PER_SECOND = this.STEPS_PER_SECOND;
        sandboxEngine.vehicles = localVehicles;

        //sandboxEngine.stepCount = this.stepCount;
        //if(this.stepCount % this.STEPS_PER_SECOND == 0 && this.stepCount > 0) {
        int currentStep = state.getStep();
        sandboxEngine.stepCount = currentStep;
        if (currentStep % this.STEPS_PER_SECOND == 0 && currentStep > 0){
            for (Vehicle v : localVehicles) {
                if(v.role.equals("EGO")){
                   v.targetSpeed =  v.targetSpeed-5 >=0? v.targetSpeed-5 : 0;
                }
                v.injectEngine(sandboxEngine);
            }
        }
        else{
            for (Vehicle v : localVehicles) {
                v.injectEngine(sandboxEngine);
            }
        }



        try {
            sandboxEngine.step();
        } catch (Exception e) {
            e.printStackTrace();
        }
        for (int i = 0; i < numsVehicles; i++) {
            Vehicle v = localVehicles.get(i);
            int offSet = i * VarTable.values().length;


            updates.add(new DataStateUpdate(offSet + VarTable.politeness.ordinal(), v.politeness));
            updates.add(new DataStateUpdate(offSet + VarTable.cooldownTimer.ordinal(), v.cooldownTimer));
            updates.add(new DataStateUpdate(offSet + VarTable.target_lane_index.ordinal(), v.getTargetLaneIndex()));
            updates.add(new DataStateUpdate(offSet + VarTable.lane_index.ordinal(), v.getLaneIndex()));
            updates.add(new DataStateUpdate(offSet + VarTable.x.ordinal(), v.x));
            updates.add(new DataStateUpdate(offSet + VarTable.y.ordinal(), v.y));
            updates.add(new DataStateUpdate(offSet + VarTable.vx.ordinal(), v.vx));
            updates.add(new DataStateUpdate(offSet + VarTable.vy.ordinal(), v.vy));
            updates.add(new DataStateUpdate(offSet + VarTable.speed.ordinal(), v.speed));
            updates.add(new DataStateUpdate(offSet + VarTable.heading.ordinal(), v.heading));
            updates.add(new DataStateUpdate(offSet + VarTable.plannedAcceleration.ordinal(), v.plannedAcceleration));
            updates.add(new DataStateUpdate(offSet + VarTable.plannedSteering.ordinal(), v.plannedSteering));
            updates.add(new DataStateUpdate(offSet + VarTable.targetSpeed.ordinal(), v.targetSpeed));
            if(sandboxEngine.crashed){
                updates.add(new DataStateUpdate(vehicles.size() * VarTable.values().length, 1.0));
            }
        }

        //this.stepCount += 1;
        if(state.getStep() == this.predictFutureSeconds * STEPS_PER_SECOND - 2) {

            try {
                Vehicle[] vs = new Vehicle[3];
                for (Vehicle v : localVehicles) {
                    if (v instanceof ProtectedControlledVehicle) {
                        vs[0] = EngineUtils.getFrontVehicle(v, sandboxEngine.vehicles, v.lane_index);
                        vs[1] = EngineUtils.getFrontVehicle(v, sandboxEngine.vehicles, v.lane_index - 1);
                        vs[2] = EngineUtils.getFrontVehicle(v, sandboxEngine.vehicles, v.lane_index + 1);
                    }
                }
                for (int i = 0; i < 3; i++) {
                    if (vs[i] != null) {
                        updates.add(new DataStateUpdate(vehicles.size() * VarTable.values().length + 1 + i, sandboxEngine.vehicles.indexOf(vs[i])));
                    } else {
                        updates.add(new DataStateUpdate(vehicles.size() * VarTable.values().length + 1 + i, -1.0));
                    }
                }
                this.finalVehicles = sandboxEngine.vehicles;

            }catch(Exception e){
                e.printStackTrace();
            }
        }


        return updates;
    }

    public static Vehicle stateToVehicle(DataState state, int vehicleIndex) {
        int offSet = vehicleIndex * VarTable.values().length;
        Vehicle v = new Vehicle();
        v.id = String.valueOf((int) state.get(offSet + VarTable.id.ordinal()));
        v.politeness = state.get(offSet + VarTable.politeness.ordinal());
        v.cooldownTimer = state.get(offSet + VarTable.cooldownTimer.ordinal());
        v.target_lane_index = (int)state.get(offSet + VarTable.target_lane_index.ordinal());
        v.lane_index = (int)state.get(offSet + VarTable.lane_index.ordinal());
        v.x = state.get(offSet + VarTable.x.ordinal());
        v.y = state.get(offSet + VarTable.y.ordinal());
        v.vx = state.get(offSet + VarTable.vx.ordinal());
        v.vy = state.get(offSet + VarTable.vy.ordinal());
        v.speed = state.get(offSet + VarTable.speed.ordinal());
        v.heading = state.get(offSet + VarTable.heading.ordinal());
        v.plannedAcceleration = state.get(offSet + VarTable.plannedAcceleration.ordinal());
        v.plannedSteering = state.get(offSet + VarTable.plannedSteering.ordinal());
        v.role = state.get(offSet + VarTable.role.ordinal()) == 0.0 ? "EGO" : "NPC";
        v.targetSpeed = state.get(offSet + VarTable.targetSpeed.ordinal());
        if(v.role.equals("EGO")){
            return new ProtectedControlledVehicle(v);
        }

        return v;
    }
}

