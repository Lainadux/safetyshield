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
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class StarkShieldApp {
    public int predictFutureSeconds = 1;

    private static final int EVOLUTION_SEQUENCE_SIZE = 10;
    //public int stepCount = 0;

    public double dt = 0;
    private HighwayEngine engine;
    private List<Vehicle> vehicles;
    public int STEPS_PER_SECOND;
    private DataState initialState;
    private ControlledSystem system;
    private EvolutionSequence sequence;

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
        SampleSet<SystemState> dss = sequence.get(this.predictFutureSeconds * STEPS_PER_SECOND - 1);
       // SampleSet<SystemState> dss = sequence.get(1);
        dss.stream().limit(5).forEach(ss -> {
            System.out.println("Summary of the evolution sequence:");
            DataState ds = ss.getDataState();
            for (int i = 0; i < vehicles.size(); i++) {
                Vehicle v = stateToVehicle(ds, i);
                System.out.println(v);
            }
        });
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


        return new DataState(vehicles.size() * VarTable.values().length, i -> values.getOrDefault(i, Double.NaN));
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


        HighwayEngine sandboxEngine = new HighwayEngine();
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
        }
        //this.stepCount += 1;


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

