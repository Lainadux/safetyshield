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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Comparator;
import java.util.Random;
import java.util.concurrent.ConcurrentHashMap;

public class StarkShieldApp {
    public int predictFutureSeconds = 1;
    public static final int auxilaryVarNums = 7;
    private static final double CRASH_DISTANCE_THRESHOLD = 0.0;
    private static final double STABILITY_DISTANCE_THRESHOLD = 0.1;
    private static final double MIN_ACCEPTABLE_ROBUSTNESS = 0.0;
    private static final double VEHICLE_LENGTH = 5.0;
    private static final double MIN_STABLE_FRONT_GAP = 15.0;
    private static final double MIN_CLOSE_FRONT_GAP = 10.0;
    private static final double MAX_STABLE_RELATIVE_SPEED = 2.0;
    private static final double SAFE_RECEDING_RELATIVE_SPEED = 2.0;
    private static final double FIRST_SECOND_SAFETY_DISTANCE_THRESHOLD = 0.1;
    private static final double CHANGE_LANE_REAR_THREAT_DISTANCE_THRESHOLD = 0.1;
    private static final double CHANGE_LANE_LOW_SPEED_DISTANCE_THRESHOLD = 0.1;
    private static final double CHANGE_LANE_MIN_SPEED = 5.0;
    private static final double FIRST_SECOND_LOOKAHEAD_TIME = 1.0;
    private static final double FIRST_SECOND_MIN_FRONT_GAP = 2.0;
    private static final double CHANGE_LANE_REAR_MAX_KARMA_LOSS = 2.0;
    private static final double MIN_FRONT_ACCELERATION_UNCERTAINTY = 0.5;
    private static final double MAX_FRONT_ACCELERATION_UNCERTAINTY = 5.0;
    private static final double FRONT_ACCELERATION_UNCERTAINTY_GAIN = 1.0;
    private static final double RANDOM_TARGET_SPEED_MEAN = 30.0;
    private static final double RANDOM_TARGET_SPEED_STD = 10.0 / 3.0;
    private static final double RANDOM_TARGET_SPEED_MIN = 20.0;
    private static final double RANDOM_TARGET_SPEED_MAX = 40.0;
    private static final double RANDOM_COOLDOWN_MEAN = 0.5;
    private static final double RANDOM_COOLDOWN_STD = 1.0 / 6.0;
    private static final double RANDOM_COOLDOWN_MIN = 0.0;
    private static final double RANDOM_COOLDOWN_MAX = 1.0;
    public static final double DEFAULT_SHIELD_EGO_RANGE_METERS = 200.0;
    public static final boolean DEFAULT_RANDOMIZE_HIDDEN_TARGET_AND_COOLDOWN = true;
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
    private double lastCollisionRobustness = Double.NaN;
    private double lastFirstSecondSafetyRobustness = Double.NaN;
    private double lastStabilityRobustness = Double.NaN;
    private double lastChangeLaneRearThreatRobustness = Double.NaN;
    private double lastChangeLaneLowSpeedRobustness = Double.NaN;
    private double lastShieldRobustness = Double.NaN;

    private List<Vehicle> filteredVehicles;
    private Map<String, Double> observedAccelerationByVehicleId = new HashMap<>();
    private Map<String, Double> frontAccelerationUncertaintyByVehicleId = new ConcurrentHashMap<>();
    private final Random hiddenStateRandom = new Random();
    private final double shieldEgoRangeMeters;
    private final boolean randomizeHiddenTargetAndCooldown;
    private final boolean checkChangeLaneToRearVehicleThreat;

    public StarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds) {
        this(engine, vehicles, predictFutureSeconds,
                DEFAULT_SHIELD_EGO_RANGE_METERS,
                DEFAULT_RANDOMIZE_HIDDEN_TARGET_AND_COOLDOWN,
                false);
    }

    public StarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                          double shieldEgoRangeMeters, boolean randomizeHiddenTargetAndCooldown) {
        this(engine, vehicles, predictFutureSeconds,
                shieldEgoRangeMeters, randomizeHiddenTargetAndCooldown, false);
    }

    public StarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                          double shieldEgoRangeMeters, boolean randomizeHiddenTargetAndCooldown,
                          boolean checkChangeLaneToRearVehicleThreat) {
        this.engine = engine;
        this.dt = engine.dt;
        this.STEPS_PER_SECOND = engine.STEPS_PER_SECOND;
        this.predictFutureSeconds = predictFutureSeconds;
        this.shieldEgoRangeMeters = shieldEgoRangeMeters;
        this.randomizeHiddenTargetAndCooldown = randomizeHiddenTargetAndCooldown;
        this.checkChangeLaneToRearVehicleThreat = checkChangeLaneToRearVehicleThreat;
        this.vehicles = vehicles;
        this.filteredVehicles = getVehiclesWithinEgoRangeIncludingEgo(shieldEgoRangeMeters);
        this.vehicles = this.filteredVehicles;
        this.observedAccelerationByVehicleId = getObservedAccelerationByVehicleId(this.vehicles);

        initialState = randomizeHiddenTargetAndCooldown
                ? this.getInitialStateWithRandomHiddenState(this.vehicles)
                : this.getInitialState(this.vehicles);
        system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(this.getEnvironmentUpdates(rg, ds)), initialState);
        sequence = new EvolutionSequence(new SilentMonitor("Vehicle"), new DefaultRandomGenerator(), rg -> system, EVOLUTION_SEQUENCE_SIZE);
        //printSummary();
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
        int firstSecondLastStep = Math.min(STEPS_PER_SECOND - 1, lastStep);
        DisTLFormula noCollision = new AlwaysDisTLFormula(
                new TargetDisTLFormula(this::resetCrashState, this::crashPenalty, CRASH_DISTANCE_THRESHOLD),
                0,
                lastStep
        );
        DisTLFormula safeFrontDistanceAtFirstSecond = new AlwaysDisTLFormula(
                new TargetDisTLFormula(this::stabilizeEgoAtFrontSafetyDistance, this::firstSecondFrontSafetyPenalty, FIRST_SECOND_SAFETY_DISTANCE_THRESHOLD),
                firstSecondLastStep,
                firstSecondLastStep
        );
        DisTLFormula stableAtLastStep = new AlwaysDisTLFormula(
                new TargetDisTLFormula(this::stabilizeEgoAgainstFrontVehicle, this::frontVehicleStabilityPenalty, STABILITY_DISTANCE_THRESHOLD),
                lastStep,
                lastStep
        );
        DisTLFormula changeLaneRearThreatAtLastStep = new AlwaysDisTLFormula(
                new TargetDisTLFormula(this::stabilizeChangeLaneRearThreat, this::changeLaneRearThreatPenalty, CHANGE_LANE_REAR_THREAT_DISTANCE_THRESHOLD),
                lastStep,
                lastStep
        );
        DisTLFormula changeLaneLowSpeedAtDecisionStep = new AlwaysDisTLFormula(
                new TargetDisTLFormula(this::stabilizeChangeLaneLowSpeed, this::changeLaneLowSpeedPenalty, CHANGE_LANE_LOW_SPEED_DISTANCE_THRESHOLD),
                0,
                0
        );
        DisTLFormula shieldCondition = new ConjunctionDisTLFormula(
                noCollision,
                new ConjunctionDisTLFormula(safeFrontDistanceAtFirstSecond, stableAtLastStep)
        );
        if (checkChangeLaneToRearVehicleThreat) {
            DisTLFormula rearThreatCondition = new ConjunctionDisTLFormula(
                    changeLaneRearThreatAtLastStep,
                    changeLaneLowSpeedAtDecisionStep
            );
            shieldCondition = new ConjunctionDisTLFormula(shieldCondition, rearThreatCondition);
        }
        DoubleSemanticsVisitor semantics = new DoubleSemanticsVisitor();
        lastCollisionRobustness = semantics.eval(noCollision).eval(EVOLUTION_SEQUENCE_SIZE, 0, sequence);
        lastFirstSecondSafetyRobustness = semantics.eval(safeFrontDistanceAtFirstSecond).eval(EVOLUTION_SEQUENCE_SIZE, 0, sequence);
        lastStabilityRobustness = semantics.eval(stableAtLastStep).eval(EVOLUTION_SEQUENCE_SIZE, 0, sequence);
        lastChangeLaneRearThreatRobustness = checkChangeLaneToRearVehicleThreat
                ? semantics.eval(changeLaneRearThreatAtLastStep).eval(EVOLUTION_SEQUENCE_SIZE, 0, sequence)
                : Double.NaN;
        lastChangeLaneLowSpeedRobustness = checkChangeLaneToRearVehicleThreat
                ? semantics.eval(changeLaneLowSpeedAtDecisionStep).eval(EVOLUTION_SEQUENCE_SIZE, 0, sequence)
                : Double.NaN;
        lastShieldRobustness = semantics.eval(shieldCondition).eval(EVOLUTION_SEQUENCE_SIZE, 0, sequence);
        return lastShieldRobustness >= MIN_ACCEPTABLE_ROBUSTNESS;
    }

    public String getUnsafeDiagnosis() {
        int lastStep = this.predictFutureSeconds * STEPS_PER_SECOND - 1;
        dss = sequence.get(lastStep);
        return dss.stream()
                .findFirst()
                .map(ss -> diagnoseState(ss.getDataState(), lastStep))
                .orElse("No prediction state available for unsafe diagnosis.");
    }

    public String getPredictedFinalVehicleState(String requestedVehicleId) {
        if (requestedVehicleId == null || requestedVehicleId.isBlank()) {
            return "No vehicle id was provided.";
        }

        int lastStep = this.predictFutureSeconds * STEPS_PER_SECOND - 1;
        SampleSet<SystemState> finalStates = sequence.get(lastStep);
        if (finalStates == null || finalStates.size() == 0) {
            return "No prediction final state is available.";
        }

        SystemState firstSample = finalStates.stream().findFirst().orElse(null);
        if (firstSample == null) {
            return "No prediction final state is available.";
        }

        DataState state = firstSample.getDataState();
        String vehicleId = requestedVehicleId.trim();
        for (int i = 0; i < vehicles.size(); i++) {
            if (vehicleId.equals(vehicleId(state, i))) {
                return formatPredictedVehicleState(state, i, lastStep, finalStates.size());
            }
        }
        return String.format("Vehicle id=%s was not found in predicted final state at step %d.", vehicleId, lastStep);
    }

    private String diagnoseState(DataState state, int lastStep) {
        int egoIndex = getEgoVehicleIndex(state);
        StringBuilder diagnosis = new StringBuilder();
        diagnosis.append(String.format("Shield diagnosis at prediction step %d: crashed=%.0f",
                lastStep, state.get(crashedIndex())));
        diagnosis.append(String.format("%n  DisTL robustness: collision=%.3f firstSecondSafety=%.3f stability=%.3f",
                lastCollisionRobustness, lastFirstSecondSafetyRobustness, lastStabilityRobustness));
        if (checkChangeLaneToRearVehicleThreat) {
            diagnosis.append(String.format(" changeLaneRearThreat=%.3f changeLaneLowSpeed=%.3f",
                    lastChangeLaneRearThreatRobustness, lastChangeLaneLowSpeedRobustness));
        }
        diagnosis.append(String.format(" shield=%.3f minAcceptable=%.3f",
                lastShieldRobustness, MIN_ACCEPTABLE_ROBUSTNESS));
        diagnosis.append(rawPenaltySummary());

        if (egoIndex < 0) {
            diagnosis.append(", ego not found");
            return diagnosis.toString();
        }

        int egoOffset = vehicleOffset(egoIndex);
        int egoLane = (int) state.get(egoOffset + VarTable.lane_index.ordinal());
        diagnosis.append(String.format(", egoLane=%d, egoX=%.2f, egoSpeed=%.2f, egoTargetSpeed=%.2f",
                egoLane,
                state.get(egoOffset + VarTable.x.ordinal()),
                state.get(egoOffset + VarTable.speed.ordinal()),
                state.get(egoOffset + VarTable.targetSpeed.ordinal())));

        for (int lane = egoLane - 1; lane <= egoLane + 1; lane++) {
            int frontIndex = getFrontVehicleIndexInLane(state, egoIndex, lane);
            if (frontIndex >= 0) {
                int frontOffset = vehicleOffset(frontIndex);
                double frontGap = state.get(frontOffset + VarTable.x.ordinal()) - state.get(egoOffset + VarTable.x.ordinal()) - VEHICLE_LENGTH;
                double closingSpeed = Math.max(0.0, state.get(egoOffset + VarTable.vx.ordinal()) - state.get(frontOffset + VarTable.vx.ordinal()));
                double penalty = frontVehicleStabilityPenalty(state, egoIndex, frontIndex);
                diagnosis.append(String.format("%n  front lane=%d vehicleId=%s vehicleIndex=%d gap=%.2f closingSpeed=%.2f penalty=%.3f",
                        lane,
                        vehicleId(state, frontIndex),
                        frontIndex,
                        frontGap,
                        closingSpeed,
                        penalty));
            }
        }

        diagnosis.append(String.format("%n  totalFrontStabilityPenalty=%.3f threshold=%.3f",
                frontVehicleStabilityPenalty(state), STABILITY_DISTANCE_THRESHOLD));
        if (checkChangeLaneToRearVehicleThreat) {
            diagnosis.append(changeLaneRearThreatDiagnosis(state, egoIndex));
        }
        return diagnosis.toString();
    }

    private String changeLaneRearThreatDiagnosis(DataState state, int egoIndex) {
        int egoOffset = vehicleOffset(egoIndex);
        int egoLane = (int) state.get(egoOffset + VarTable.lane_index.ordinal());
        int egoTargetLane = (int) state.get(egoOffset + VarTable.target_lane_index.ordinal());
        boolean isChangeLane = state.get(isChangeLaneIndex()) > 0.0;
        StringBuilder diagnosis = new StringBuilder();
        diagnosis.append(String.format("%n  changeLaneRearThreat enabled=true isChangeLane=%s egoLane=%d egoTargetLane=%d",
                isChangeLane, egoLane, egoTargetLane));
        if (!isChangeLane) {
            return diagnosis.toString();
        }
        diagnosis.append(String.format(" lowSpeedPenalty=%.3f minChangeLaneSpeed=%.2f",
                changeLaneLowSpeedPenalty(state), CHANGE_LANE_MIN_SPEED));

        int rearIndex = getConfiguredRearThreatRearVehicleIndex(state);
        if (rearIndex < 0) {
            diagnosis.append("%n  changeLaneRearThreat rear=none rawPenalty=0.000");
            return diagnosis.toString();
        }

        int rearOffset = vehicleOffset(rearIndex);
        int rearFrontIndexBeforeCutIn = getFrontVehicleIndexInLaneExcluding(state, rearIndex,
                (int) state.get(rearOffset + VarTable.lane_index.ordinal()), egoIndex);
        double beforeAccel = state.get(rearThreatBeforeAccelerationIndex());
        double afterAccel = rawIdmAcceleration(state, rearIndex, egoIndex);
        double karma = afterAccel - beforeAccel;
        double karmaLoss = Math.max(0.0, -karma);
        double rawPenalty = changeLaneRearThreatPenalty(state);
        DataState targetState = stabilizeChangeLaneRearThreat(null, state);
        double targetPenalty = changeLaneRearThreatPenalty(targetState);
        double desiredEgoX = targetState.get(egoOffset + VarTable.x.ordinal());
        double egoX = state.get(egoOffset + VarTable.x.ordinal());
        double rearGap = egoX - state.get(rearOffset + VarTable.x.ordinal()) - VEHICLE_LENGTH;
        diagnosis.append(String.format(
                "%n  changeLaneRearThreat rearId=%s rearIndex=%d rearLane=%d rearGap=%.2f rearVx=%.2f egoX=%.2f egoVx=%.2f beforeFrontId=%s beforeAccel=%.3f afterAccel=%.3f karma=%.3f karmaLoss=%.3f maxAllowedLoss=%.3f rawPenalty=%.3f targetPenalty=%.3f desiredEgoX=%.2f desiredDeltaX=%.2f",
                vehicleId(state, rearIndex),
                rearIndex,
                (int) state.get(rearOffset + VarTable.lane_index.ordinal()),
                rearGap,
                state.get(rearOffset + VarTable.vx.ordinal()),
                egoX,
                state.get(egoOffset + VarTable.vx.ordinal()),
                rearFrontIndexBeforeCutIn >= 0 ? vehicleId(state, rearFrontIndexBeforeCutIn) : "none",
                beforeAccel,
                afterAccel,
                karma,
                karmaLoss,
                CHANGE_LANE_REAR_MAX_KARMA_LOSS,
                rawPenalty,
                targetPenalty,
                desiredEgoX,
                desiredEgoX - egoX
        ));
        return diagnosis.toString();
    }

    private String formatPredictedVehicleState(DataState state, int vehicleIndex, int predictionStep, int sampleCount) {
        int offset = vehicleOffset(vehicleIndex);
        return String.format(
                "Predicted final vehicle state: id=%s vehicleIndex=%d predictionStep=%d sample=first/%d role=%s lane=%d targetLane=%d x=%.2f y=%.2f vx=%.2f vy=%.2f speed=%.2f targetSpeed=%.2f heading=%.3f plannedAcceleration=%.2f plannedSteering=%.3f",
                vehicleId(state, vehicleIndex),
                vehicleIndex,
                predictionStep,
                sampleCount,
                state.get(offset + VarTable.role.ordinal()) == 0.0 ? "EGO" : "NPC",
                (int) state.get(offset + VarTable.lane_index.ordinal()),
                (int) state.get(offset + VarTable.target_lane_index.ordinal()),
                state.get(offset + VarTable.x.ordinal()),
                state.get(offset + VarTable.y.ordinal()),
                state.get(offset + VarTable.vx.ordinal()),
                state.get(offset + VarTable.vy.ordinal()),
                state.get(offset + VarTable.speed.ordinal()),
                state.get(offset + VarTable.targetSpeed.ordinal()),
                state.get(offset + VarTable.heading.ordinal()),
                state.get(offset + VarTable.plannedAcceleration.ordinal()),
                state.get(offset + VarTable.plannedSteering.ordinal())
        );
    }

    private String rawPenaltySummary() {
        if (dss == null || dss.size() == 0) {
            return "";
        }
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        double sum = 0.0;
        int count = 0;
        for (SystemState systemState : dss.stream().toList()) {
            double penalty = frontVehicleStabilityPenalty(systemState.getDataState());
            min = Math.min(min, penalty);
            max = Math.max(max, penalty);
            sum += penalty;
            count++;
        }
        return String.format("%n  Raw final penalty samples: count=%d min=%.3f avg=%.3f max=%.3f threshold=%.3f",
                count, min, sum / count, max, STABILITY_DISTANCE_THRESHOLD);
    }

    private String vehicleId(DataState state, int vehicleIndex) {
        int offset = vehicleOffset(vehicleIndex);
        return String.valueOf((int) state.get(offset + VarTable.id.ordinal()));
    }

    private DataState resetCrashState(RandomGenerator rg, DataState state) {
        return state.apply(List.of(new DataStateUpdate(crashedIndex(), 0.0)));
    }

    private DataState stabilizeEgoAgainstFrontVehicle(RandomGenerator rg, DataState state) {
        int egoIndex = getEgoVehicleIndex(state);
        if (egoIndex < 0) {
            return state;
        }

        List<Integer> adjacentVehicles = getVehiclesInAdjacentLanes(state, egoIndex);
        if (adjacentVehicles.isEmpty()) {
            return state;
        }

        int egoOffset = vehicleOffset(egoIndex);
        double targetX = Double.POSITIVE_INFINITY;
        double targetVx = Double.POSITIVE_INFINITY;
        double targetSpeed = Double.POSITIVE_INFINITY;

        for (int vehicleIndex : adjacentVehicles) {
            int offset = vehicleOffset(vehicleIndex);
            double vehicleX = state.get(offset + VarTable.x.ordinal());
            double vehicleVx = state.get(offset + VarTable.vx.ordinal());
            double vehicleSpeed = state.get(offset + VarTable.speed.ordinal());
            targetX = Math.min(targetX, vehicleX - VEHICLE_LENGTH - MIN_STABLE_FRONT_GAP);
            targetVx = Math.min(targetVx, vehicleVx);
            targetSpeed = Math.min(targetSpeed, vehicleSpeed);
        }

        return state.apply(List.of(
                new DataStateUpdate(egoOffset + VarTable.x.ordinal(), targetX),
                new DataStateUpdate(egoOffset + VarTable.vx.ordinal(), targetVx),
                new DataStateUpdate(egoOffset + VarTable.speed.ordinal(), targetSpeed),
                new DataStateUpdate(egoOffset + VarTable.targetSpeed.ordinal(), targetSpeed)
        ));
    }

    private DataState stabilizeEgoAtFrontSafetyDistance(RandomGenerator rg, DataState state) {
        int egoIndex = getEgoVehicleIndex(state);
        if (egoIndex < 0) {
            return state;
        }

        int egoOffset = vehicleOffset(egoIndex);
        int egoLane = (int) state.get(egoOffset + VarTable.lane_index.ordinal());
        int frontIndex = getFrontVehicleIndexInLane(state, egoIndex, egoLane);
        if (frontIndex < 0) {
            return state;
        }

        int frontOffset = vehicleOffset(frontIndex);
        double frontX = state.get(frontOffset + VarTable.x.ordinal());
        double egoVx = state.get(egoOffset + VarTable.vx.ordinal());
        double frontVx = state.get(frontOffset + VarTable.vx.ordinal());
        double egoAcceleration = state.get(egoOffset + VarTable.plannedAcceleration.ordinal());
        double frontAcceleration = state.get(frontOffset + VarTable.plannedAcceleration.ordinal());
        double frontAccelerationUncertainty = getFrontAccelerationUncertainty(state, frontIndex);
        double safetyGap = calculateOneSecondWorstCaseSafetyGap(egoVx, frontVx, egoAcceleration, frontAcceleration, frontAccelerationUncertainty);
        return state.apply(List.of(
                new DataStateUpdate(egoOffset + VarTable.x.ordinal(), frontX - VEHICLE_LENGTH - safetyGap)
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

    private double firstSecondFrontSafetyPenalty(DataState state) {
        int egoIndex = getEgoVehicleIndex(state);
        if (egoIndex < 0) {
            return 0.0;
        }

        int egoOffset = vehicleOffset(egoIndex);
        int egoLane = (int) state.get(egoOffset + VarTable.lane_index.ordinal());
        int frontIndex = getFrontVehicleIndexInLane(state, egoIndex, egoLane);
        if (frontIndex < 0) {
            return 0.0;
        }

        int frontOffset = vehicleOffset(frontIndex);
        double egoX = state.get(egoOffset + VarTable.x.ordinal());
        double frontX = state.get(frontOffset + VarTable.x.ordinal());
        double egoVx = state.get(egoOffset + VarTable.vx.ordinal());
        double frontVx = state.get(frontOffset + VarTable.vx.ordinal());
        double egoAcceleration = state.get(egoOffset + VarTable.plannedAcceleration.ordinal());
        double frontAcceleration = state.get(frontOffset + VarTable.plannedAcceleration.ordinal());
        double frontGap = frontX - egoX - VEHICLE_LENGTH;
        double frontAccelerationUncertainty = getFrontAccelerationUncertainty(state, frontIndex);
        double safetyGap = calculateOneSecondWorstCaseSafetyGap(egoVx, frontVx, egoAcceleration, frontAcceleration, frontAccelerationUncertainty);
        if (safetyGap <= 0.0) {
            return 0.0;
        }
        return Math.min(1.0, Math.max(0.0, safetyGap - frontGap) / safetyGap);
    }

    private DataState stabilizeChangeLaneLowSpeed(RandomGenerator rg, DataState state) {
        if (state.get(isChangeLaneIndex()) <= 0.0) {
            return state;
        }
        int egoIndex = getEgoVehicleIndex(state);
        if (egoIndex < 0) {
            return state;
        }
        int egoOffset = vehicleOffset(egoIndex);
        return state.apply(List.of(
                new DataStateUpdate(egoOffset + VarTable.vx.ordinal(), Math.max(state.get(egoOffset + VarTable.vx.ordinal()), CHANGE_LANE_MIN_SPEED)),
                new DataStateUpdate(egoOffset + VarTable.speed.ordinal(), Math.max(state.get(egoOffset + VarTable.speed.ordinal()), CHANGE_LANE_MIN_SPEED)),
                new DataStateUpdate(egoOffset + VarTable.targetSpeed.ordinal(), Math.max(state.get(egoOffset + VarTable.targetSpeed.ordinal()), CHANGE_LANE_MIN_SPEED))
        ));
    }

    private double changeLaneLowSpeedPenalty(DataState state) {
        if (state.get(isChangeLaneIndex()) <= 0.0) {
            return 0.0;
        }
        int egoIndex = getEgoVehicleIndex(state);
        if (egoIndex < 0) {
            return 0.0;
        }
        double egoSpeed = state.get(vehicleOffset(egoIndex) + VarTable.speed.ordinal());
        if (egoSpeed >= CHANGE_LANE_MIN_SPEED) {
            return 0.0;
        }
        return Math.min(1.0, (CHANGE_LANE_MIN_SPEED - egoSpeed) / CHANGE_LANE_MIN_SPEED);
    }

    private DataState stabilizeChangeLaneRearThreat(RandomGenerator rg, DataState state) {
        if (state.get(isChangeLaneIndex()) <= 0.0) {
            return state;
        }
        int egoIndex = getEgoVehicleIndex(state);
        int rearIndex = getConfiguredRearThreatRearVehicleIndex(state);
        if (egoIndex < 0 || rearIndex < 0) {
            return state;
        }

        int egoOffset = vehicleOffset(egoIndex);
        int rearOffset = vehicleOffset(rearIndex);
        int rearLane = (int) state.get(rearOffset + VarTable.lane_index.ordinal());
        double beforeAccel = state.get(rearThreatBeforeAccelerationIndex());
        double requiredAfterCutInAcceleration = beforeAccel - CHANGE_LANE_REAR_MAX_KARMA_LOSS;
        double desiredEgoX = desiredFrontXForRearAcceleration(state, rearIndex, egoIndex, requiredAfterCutInAcceleration);

        List<DataStateUpdate> updates = new ArrayList<>();
        updates.add(new DataStateUpdate(egoOffset + VarTable.x.ordinal(), desiredEgoX));
        updates.add(new DataStateUpdate(egoOffset + VarTable.lane_index.ordinal(), rearLane));
        updates.add(new DataStateUpdate(egoOffset + VarTable.target_lane_index.ordinal(), rearLane));

        for (int i = 0; i < vehicles.size(); i++) {
            if (i == egoIndex || i == rearIndex) {
                continue;
            }
            int offset = vehicleOffset(i);
            updates.add(new DataStateUpdate(offset + VarTable.lane_index.ordinal(), -999.0));
            updates.add(new DataStateUpdate(offset + VarTable.target_lane_index.ordinal(), -999.0));
        }
        return state.apply(updates);
    }

    private double changeLaneRearThreatPenalty(DataState state) {
        int egoIndex = getEgoVehicleIndex(state);
        int rearIndex = getConfiguredRearThreatRearVehicleIndex(state);
        if (egoIndex < 0 || rearIndex < 0) {
            return 0.0;
        }

        double rearAccelerationBeforeCutIn = state.get(rearThreatBeforeAccelerationIndex());
        double rearAccelerationAfterCutIn = rawIdmAcceleration(state, rearIndex, egoIndex);
        double karma = rearAccelerationAfterCutIn - rearAccelerationBeforeCutIn;
        double karmaLoss = Math.max(0.0, -karma);
        if (karmaLoss <= CHANGE_LANE_REAR_MAX_KARMA_LOSS) {
            return 0.0;
        }
        return Math.min(1.0, (karmaLoss - CHANGE_LANE_REAR_MAX_KARMA_LOSS) / CHANGE_LANE_REAR_MAX_KARMA_LOSS);
    }

    private double rawIdmAcceleration(DataState state, int vehicleIndex, int frontVehicleIndex) {
        int vehicleOffset = vehicleOffset(vehicleIndex);
        double v = state.get(vehicleOffset + VarTable.vx.ordinal());
        double v0 = Math.max(0.1, state.get(vehicleOffset + VarTable.targetSpeed.ordinal()));
        double freeFlowTerm = 1.0 - Math.pow(v / v0, 4.0);
        if (frontVehicleIndex < 0) {
            return 3.0 * freeFlowTerm;
        }

        int frontOffset = vehicleOffset(frontVehicleIndex);
        double frontVx = state.get(frontOffset + VarTable.vx.ordinal());
        double gap = state.get(frontOffset + VarTable.x.ordinal()) - state.get(vehicleOffset + VarTable.x.ordinal()) - VEHICLE_LENGTH;
        gap = Math.max(gap, 0.01);
        double dv = v - frontVx;
        double desiredGap = 10.0 + v * 1.5 + (v * dv) / (2.0 * Math.sqrt(3.0 * 5.0));
        desiredGap = Math.max(desiredGap, 10.0);
        return 3.0 * (freeFlowTerm - Math.pow(desiredGap / gap, 2.0));
    }

    private double desiredFrontXForRearAcceleration(DataState state, int rearIndex, int frontIndex,
                                                    double requiredAcceleration) {
        int rearOffset = vehicleOffset(rearIndex);
        int frontOffset = vehicleOffset(frontIndex);
        double rearX = state.get(rearOffset + VarTable.x.ordinal());
        double rearVx = state.get(rearOffset + VarTable.vx.ordinal());
        double frontVx = state.get(frontOffset + VarTable.vx.ordinal());
        double rearTargetSpeed = Math.max(0.1, state.get(rearOffset + VarTable.targetSpeed.ordinal()));
        double freeFlowTerm = 1.0 - Math.pow(rearVx / rearTargetSpeed, 4.0);
        double requiredInteractionTerm = freeFlowTerm - requiredAcceleration / 3.0;
        if (requiredInteractionTerm <= 0.0) {
            return Math.max(state.get(frontOffset + VarTable.x.ordinal()), rearX + VEHICLE_LENGTH + 10.0);
        }

        double dv = rearVx - frontVx;
        double desiredGap = 10.0 + rearVx * 1.5 + (rearVx * dv) / (2.0 * Math.sqrt(3.0 * 5.0));
        desiredGap = Math.max(desiredGap, 10.0);
        double requiredGap = desiredGap / Math.sqrt(requiredInteractionTerm);
        double currentFrontX = state.get(frontOffset + VarTable.x.ordinal());
        return Math.max(currentFrontX, rearX + VEHICLE_LENGTH + requiredGap);
    }

    private double calculateOneSecondWorstCaseSafetyGap(double egoVx, double frontVx, double egoAcceleration,
                                                       double frontAcceleration, double frontAccelerationUncertainty) {
        double egoSpeed = Math.max(0.0, egoVx);
        double frontSpeed = Math.max(0.0, frontVx);
        double frontWorstAcceleration = frontAcceleration - frontAccelerationUncertainty;
        double relativeClosingDistance =
                (egoSpeed - frontSpeed) * FIRST_SECOND_LOOKAHEAD_TIME
                        + 0.5 * (egoAcceleration - frontWorstAcceleration)
                        * FIRST_SECOND_LOOKAHEAD_TIME * FIRST_SECOND_LOOKAHEAD_TIME;
        return FIRST_SECOND_MIN_FRONT_GAP + Math.max(0.0, relativeClosingDistance);
    }

    private double getFrontAccelerationUncertainty(DataState state, int frontIndex) {
        return frontAccelerationUncertaintyByVehicleId.getOrDefault(
                vehicleId(state, frontIndex),
                MIN_FRONT_ACCELERATION_UNCERTAINTY
        );
    }

    private double frontVehicleStabilityPenalty(DataState state, int egoIndex, int frontIndex) {
        int egoOffset = vehicleOffset(egoIndex);
        int frontOffset = vehicleOffset(frontIndex);
        double egoX = state.get(egoOffset + VarTable.x.ordinal());
        double frontX = state.get(frontOffset + VarTable.x.ordinal());
        double egoVx = state.get(egoOffset + VarTable.vx.ordinal());
        double frontVx = state.get(frontOffset + VarTable.vx.ordinal());
        double frontGap = frontX - egoX - VEHICLE_LENGTH;
        double relativeSpeed = egoVx - frontVx;
        double closingSpeed = Math.max(0.0, relativeSpeed);
        int egoLane = (int) state.get(egoOffset + VarTable.lane_index.ordinal());
        int frontLane = (int) state.get(frontOffset + VarTable.lane_index.ordinal());
        if (frontGap <= 0.0 && frontLane == egoLane) {
            return 1.0;
        }
        if (frontGap >= MIN_CLOSE_FRONT_GAP && closingSpeed == 0.0) {
            return 0.0;
        }
        if (frontGap < MIN_CLOSE_FRONT_GAP && relativeSpeed <= -SAFE_RECEDING_RELATIVE_SPEED) {
            return 0.0;
        }
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

    private List<Integer> getVehiclesInAdjacentLanes(DataState state, int egoIndex) {
        List<Integer> adjacentVehicles = new ArrayList<>();
        if (egoIndex < 0) {
            return adjacentVehicles;
        }

        int egoOffset = vehicleOffset(egoIndex);
        int egoLane = (int) state.get(egoOffset + VarTable.lane_index.ordinal());
        for (int i = 0; i < vehicles.size(); i++) {
            if (i == egoIndex) {
                continue;
            }
            int offset = vehicleOffset(i);
            int lane = (int) state.get(offset + VarTable.lane_index.ordinal());
            if (lane >= egoLane - 1 && lane <= egoLane + 1) {
                adjacentVehicles.add(i);
            }
        }
        return adjacentVehicles;
    }

    private int getFrontVehicleIndexInLane(DataState state, int egoIndex, int targetLane) {
        return getFrontVehicleIndexInLaneExcluding(state, egoIndex, targetLane, -1);
    }

    private int getFrontVehicleIndexInLaneExcluding(DataState state, int egoIndex, int targetLane, int excludedIndex) {
        if (egoIndex < 0) {
            return -1;
        }

        int egoOffset = vehicleOffset(egoIndex);
        double egoX = state.get(egoOffset + VarTable.x.ordinal());
        int frontIndex = -1;
        double closestFrontX = Double.POSITIVE_INFINITY;
        for (int i = 0; i < vehicles.size(); i++) {
            if (i == egoIndex || i == excludedIndex) {
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

    private int getChangeLaneTargetRearVehicleIndex(DataState state, int egoIndex) {
        if (egoIndex < 0 || state.get(isChangeLaneIndex()) <= 0.0) {
            return -1;
        }
        int egoOffset = vehicleOffset(egoIndex);
        int targetLane = (int) state.get(egoOffset + VarTable.target_lane_index.ordinal());
        return getRearVehicleIndexInLane(state, egoIndex, targetLane);
    }

    private int getRearVehicleIndexInLane(DataState state, int egoIndex, int targetLane) {
        if (egoIndex < 0) {
            return -1;
        }

        int egoOffset = vehicleOffset(egoIndex);
        double egoX = state.get(egoOffset + VarTable.x.ordinal());
        int rearIndex = -1;
        double closestRearX = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < vehicles.size(); i++) {
            if (i == egoIndex) {
                continue;
            }
            int offset = vehicleOffset(i);
            int lane = (int) state.get(offset + VarTable.lane_index.ordinal());
            double x = state.get(offset + VarTable.x.ordinal());
            if (lane == targetLane && x < egoX && x > closestRearX) {
                rearIndex = i;
                closestRearX = x;
            }
        }
        return rearIndex;
    }

    private int vehicleOffset(int vehicleIndex) {
        return vehicleIndex * VarTable.values().length;
    }

    private int crashedIndex() {
        return vehicles.size() * VarTable.values().length;
    }

    private int isChangeLaneIndex() {
        return vehicles.size() * VarTable.values().length + 4;
    }

    private int rearThreatRearIndexIndex() {
        return vehicles.size() * VarTable.values().length + 5;
    }

    private int rearThreatBeforeAccelerationIndex() {
        return vehicles.size() * VarTable.values().length + 6;
    }

    private int getConfiguredRearThreatRearVehicleIndex(DataState state) {
        int rearIndex = (int) Math.round(state.get(rearThreatRearIndexIndex()));
        if (rearIndex < 0 || rearIndex >= vehicles.size() || state.get(isChangeLaneIndex()) <= 0.0) {
            return -1;
        }
        return rearIndex;
    }

    public List<Vehicle> getVehiclesWithinEgoRange(double rangeMeters) {
        Vehicle ego = getEgoVehicle();
        if (ego == null) {
            return List.of();
        }
        return vehicles.stream()
                .filter(vehicle -> vehicle != ego)
                .filter(vehicle -> Math.abs(vehicle.x - ego.x) <= rangeMeters)
                .sorted(Comparator.comparingDouble(vehicle -> Math.abs(vehicle.x - ego.x)))
                .toList();
    }

    public List<Vehicle> getVehiclesWithinOneHundredMetersOfEgo() {
        return getVehiclesWithinEgoRange(100.0);
    }

    private List<Vehicle> getVehiclesWithinEgoRangeIncludingEgo(double rangeMeters) {
        Vehicle ego = getEgoVehicle();
        if (ego == null) {
            return List.of();
        }
        List<Vehicle> nearbyVehicles = new LinkedList<>();
        nearbyVehicles.add(ego);
        nearbyVehicles.addAll(getVehiclesWithinEgoRange(rangeMeters));
        return nearbyVehicles;
    }

    private Vehicle getEgoVehicle() {
        for (Vehicle vehicle : vehicles) {
            if ("EGO".equals(vehicle.role)) {
                return vehicle;
            }
        }
        return null;
    }

    private DataState getInitialState(List<Vehicle> vehicles) {
//        System.out.println("initial state fetched by stark:");
//        for (Vehicle v : vehicles) {
//
//            System.out.println(v);
//        }

        Map<Integer, Double> values = new HashMap<>();
        for (int i = 0; i < vehicles.size(); i++) {
            Vehicle v = vehicles.get(i);
            int offSet = i * VarTable.values().length;
            values.put(offSet + VarTable.id.ordinal(),Double.valueOf(v.id));
            //values.put(offSet + VarTable.politeness.ordinal(), v.politeness);
            values.put(offSet + VarTable.politeness.ordinal(), 0.0);
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
//            if(v.role.equals("EGO")){
//                System.out.println("starked ego intention targetspeed: " + v.targetSpeed + ", current speed: " + v.speed);
//            }
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
        //5. whether the current ego decision is a lane-change action
        values.put(isChangeLaneIndex(), initialIsChangeLane(vehicles) ? 1.0 : 0.0);
        populateRearThreatAuxiliaryValues(values, vehicles);


//        return new DataState(vehicles.size() * VarTable.values().length + 1,
//                i -> values.getOrDefault(i, Double.NaN));
        return new DataState(values.size(),i -> values.getOrDefault(i, Double.NaN));
    }

    private DataState getInitialStateWithRandomHiddenState(List<Vehicle> vehicles) {
        Map<Integer, Double> values = new HashMap<>();
        for (int i = 0; i < vehicles.size(); i++) {
            Vehicle v = vehicles.get(i);
            int offSet = i * VarTable.values().length;
            values.put(offSet + VarTable.id.ordinal(), Double.valueOf(v.id));
            //values.put(offSet + VarTable.politeness.ordinal(), v.politeness);
            values.put(offSet + VarTable.politeness.ordinal(), 0.0);
            values.put(offSet + VarTable.cooldownTimer.ordinal(), getRandomCooldownTimer(v));
            values.put(offSet + VarTable.target_lane_index.ordinal(), (double) v.getTargetLaneIndex());
            values.put(offSet + VarTable.lane_index.ordinal(), (double) v.getLaneIndex());
            values.put(offSet + VarTable.x.ordinal(), v.x);
            values.put(offSet + VarTable.y.ordinal(), v.y);
            values.put(offSet + VarTable.vx.ordinal(), v.vx);
            values.put(offSet + VarTable.vy.ordinal(), v.vy);
            values.put(offSet + VarTable.speed.ordinal(), v.speed);
            values.put(offSet + VarTable.heading.ordinal(), v.heading);
            values.put(offSet + VarTable.plannedAcceleration.ordinal(), v.plannedAcceleration);
            values.put(offSet + VarTable.plannedSteering.ordinal(), v.plannedSteering);
            values.put(offSet + VarTable.role.ordinal(), v.role.equals("EGO") ? 0.0 : 1.0);
            values.put(offSet + VarTable.targetSpeed.ordinal(), getRandomTargetSpeed(v));
        }
        values.put(vehicles.size() * VarTable.values().length, 0.0);
        values.put(vehicles.size() * VarTable.values().length + 1, -1.0);
        values.put(vehicles.size() * VarTable.values().length + 2, -1.0);
        values.put(vehicles.size() * VarTable.values().length + 3, -1.0);
        values.put(isChangeLaneIndex(), initialIsChangeLane(vehicles) ? 1.0 : 0.0);
        populateRearThreatAuxiliaryValues(values, vehicles);
        return new DataState(values.size(), i -> values.getOrDefault(i, Double.NaN));
    }

    private void populateRearThreatAuxiliaryValues(Map<Integer, Double> values, List<Vehicle> vehicles) {
        int egoIndex = initialEgoIndex(values, vehicles.size());
        if (egoIndex < 0 || !initialIsChangeLane(vehicles)) {
            values.put(rearThreatRearIndexIndex(), -1.0);
            values.put(rearThreatBeforeAccelerationIndex(), 0.0);
            return;
        }

        int egoOffset = vehicleOffset(egoIndex);
        int targetLane = values.get(egoOffset + VarTable.target_lane_index.ordinal()).intValue();
        int rearIndex = initialRearVehicleIndexInLane(values, vehicles.size(), egoIndex, targetLane);
        if (rearIndex < 0) {
            values.put(rearThreatRearIndexIndex(), -1.0);
            values.put(rearThreatBeforeAccelerationIndex(), 0.0);
            return;
        }

        int rearFrontIndex = initialFrontVehicleIndexInLaneExcluding(values, vehicles.size(), rearIndex, targetLane, egoIndex);
        values.put(rearThreatRearIndexIndex(), (double) rearIndex);
        values.put(rearThreatBeforeAccelerationIndex(), initialRawIdmAcceleration(values, rearIndex, rearFrontIndex));
    }

    private int initialEgoIndex(Map<Integer, Double> values, int vehicleCount) {
        for (int i = 0; i < vehicleCount; i++) {
            int offset = vehicleOffset(i);
            if (values.get(offset + VarTable.role.ordinal()) == 0.0) {
                return i;
            }
        }
        return -1;
    }

    private int initialRearVehicleIndexInLane(Map<Integer, Double> values, int vehicleCount, int egoIndex, int targetLane) {
        int egoOffset = vehicleOffset(egoIndex);
        double egoX = values.get(egoOffset + VarTable.x.ordinal());
        int rearIndex = -1;
        double closestRearX = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < vehicleCount; i++) {
            if (i == egoIndex) {
                continue;
            }
            int offset = vehicleOffset(i);
            int lane = values.get(offset + VarTable.lane_index.ordinal()).intValue();
            double x = values.get(offset + VarTable.x.ordinal());
            if (lane == targetLane && x < egoX && x > closestRearX) {
                rearIndex = i;
                closestRearX = x;
            }
        }
        return rearIndex;
    }

    private int initialFrontVehicleIndexInLaneExcluding(Map<Integer, Double> values, int vehicleCount,
                                                        int vehicleIndex, int targetLane, int excludedIndex) {
        int vehicleOffset = vehicleOffset(vehicleIndex);
        double vehicleX = values.get(vehicleOffset + VarTable.x.ordinal());
        int frontIndex = -1;
        double closestFrontX = Double.POSITIVE_INFINITY;
        for (int i = 0; i < vehicleCount; i++) {
            if (i == vehicleIndex || i == excludedIndex) {
                continue;
            }
            int offset = vehicleOffset(i);
            int lane = values.get(offset + VarTable.lane_index.ordinal()).intValue();
            double x = values.get(offset + VarTable.x.ordinal());
            if (lane == targetLane && x > vehicleX && x < closestFrontX) {
                frontIndex = i;
                closestFrontX = x;
            }
        }
        return frontIndex;
    }

    private double initialRawIdmAcceleration(Map<Integer, Double> values, int vehicleIndex, int frontVehicleIndex) {
        int vehicleOffset = vehicleOffset(vehicleIndex);
        double v = values.get(vehicleOffset + VarTable.vx.ordinal());
        double v0 = Math.max(0.1, values.get(vehicleOffset + VarTable.targetSpeed.ordinal()));
        double freeFlowTerm = 1.0 - Math.pow(v / v0, 4.0);
        if (frontVehicleIndex < 0) {
            return 3.0 * freeFlowTerm;
        }

        int frontOffset = vehicleOffset(frontVehicleIndex);
        double frontVx = values.get(frontOffset + VarTable.vx.ordinal());
        double gap = values.get(frontOffset + VarTable.x.ordinal())
                - values.get(vehicleOffset + VarTable.x.ordinal())
                - VEHICLE_LENGTH;
        gap = Math.max(gap, 0.01);
        double dv = v - frontVx;
        double desiredGap = 10.0 + v * 1.5 + (v * dv) / (2.0 * Math.sqrt(3.0 * 5.0));
        desiredGap = Math.max(desiredGap, 10.0);
        return 3.0 * (freeFlowTerm - Math.pow(desiredGap / gap, 2.0));
    }

    private boolean initialIsChangeLane(List<Vehicle> vehicles) {
        for (Vehicle vehicle : vehicles) {
            if ("EGO".equals(vehicle.role)) {
                return vehicle.getLaneIndex() != vehicle.getTargetLaneIndex();
            }
        }
        return false;
    }

    private double getRandomTargetSpeed(Vehicle vehicle) {
        if ("EGO".equals(vehicle.role)) {
            return vehicle.targetSpeed;
        }
        return clippedGaussian(RANDOM_TARGET_SPEED_MEAN, RANDOM_TARGET_SPEED_STD,
                RANDOM_TARGET_SPEED_MIN, RANDOM_TARGET_SPEED_MAX);
    }

    private double getRandomCooldownTimer(Vehicle vehicle) {
        if ("EGO".equals(vehicle.role)) {
            return vehicle.cooldownTimer;
        }
        return clippedGaussian(RANDOM_COOLDOWN_MEAN, RANDOM_COOLDOWN_STD,
                RANDOM_COOLDOWN_MIN, RANDOM_COOLDOWN_MAX);
    }

    private double clippedGaussian(double mean, double std, double min, double max) {
        double value = mean + hiddenStateRandom.nextGaussian() * std;
        return Math.max(min, Math.min(max, value));
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
        if (currentStep == 0) {
            recordFirstInternalAccelerationUncertainty(localVehicles);
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

    private Map<String, Double> getObservedAccelerationByVehicleId(List<Vehicle> vehicles) {
        Map<String, Double> observedAccelerations = new HashMap<>();
        for (Vehicle vehicle : vehicles) {
            observedAccelerations.put(vehicle.id, vehicle.plannedAcceleration);
        }
        return observedAccelerations;
    }

    private void recordFirstInternalAccelerationUncertainty(List<Vehicle> localVehicles) {
        for (Vehicle vehicle : localVehicles) {
            Double observedAcceleration = observedAccelerationByVehicleId.get(vehicle.id);
            if (observedAcceleration == null) {
                continue;
            }
            double accelerationDifference = Math.abs(observedAcceleration - vehicle.plannedAcceleration);
            double uncertainty = MIN_FRONT_ACCELERATION_UNCERTAINTY
                    + FRONT_ACCELERATION_UNCERTAINTY_GAIN * accelerationDifference;
            uncertainty = Math.max(MIN_FRONT_ACCELERATION_UNCERTAINTY,
                    Math.min(MAX_FRONT_ACCELERATION_UNCERTAINTY, uncertainty));
            frontAccelerationUncertaintyByVehicleId.putIfAbsent(vehicle.id, uncertainty);
        }
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

