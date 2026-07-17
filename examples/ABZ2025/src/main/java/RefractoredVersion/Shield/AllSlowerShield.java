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

package RefractoredVersion.Shield;

import RefractoredVersion.Engine.JavaHighwayEngine;
import RefractoredVersion.Engine.JavaHighwayEngineUtils;
import RefractoredVersion.Engine.NonNpcVehicle;
import RefractoredVersion.Engine.SandboxJavaHighwayEngine;
import RefractoredVersion.Engine.Vehicle;
import Scenarios.Engine.VarTable;
import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.SampleSet;
import it.unicam.quasylab.jspear.SystemState;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.distl.AlwaysDisTLFormula;
import it.unicam.quasylab.jspear.distl.ConjunctionDisTLFormula;
import it.unicam.quasylab.jspear.distl.DisTLFormula;
import it.unicam.quasylab.jspear.distl.DoubleSemanticsVisitor;
import it.unicam.quasylab.jspear.distl.TargetDisTLFormula;
import nl.tue.Monitoring.PerceivedSystemState;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AllSlowerShield {
    private static final int DEFAULT_PREDICTION_TIME = 3;
    private static final int AUXILIARY_VAR_NUMS = 7;
    private static final int EVOLUTION_SEQUENCE_SIZE = 30;
    private static final double CRASH_DISTANCE_THRESHOLD = 0.0;
    private static final double FIRST_SECOND_SAFETY_DISTANCE_THRESHOLD = 0.1;
    private static final double STABILITY_DISTANCE_THRESHOLD = 0.1;
    private static final double MIN_ACCEPTABLE_ROBUSTNESS = 0.0;
    private static final double VEHICLE_LENGTH = 5.0;
    private static final double MIN_STABLE_FRONT_GAP = 15.0;
    private static final double MIN_CLOSE_FRONT_GAP = 10.0;
    private static final double MAX_STABLE_RELATIVE_SPEED = 2.0;
    private static final double SAFE_RECEDING_RELATIVE_SPEED = 2.0;
    private static final double AGGRESSIVE_V3_TTC_THRESHOLD = 3.0;
    private static final double FIRST_SECOND_LOOKAHEAD_TIME = 1.0;
    private static final double FIRST_SECOND_MIN_FRONT_GAP = 2.0;
    private static final double MIN_FRONT_ACCELERATION_UNCERTAINTY = 0.5;
    private static final double FIXED_PREDICTION_RANDOM_TARGET_DELTA = 1.0;

    private final JavaHighwayEngine sourceEngine;
    private final int predictionTime;
    private SandboxJavaHighwayEngine sandboxEngine;
    private EvolutionSequence sequence;
    private double lastCollisionRobustness = Double.NaN;
    private double lastFirstSecondSafetyRobustness = Double.NaN;
    private double lastStabilityRobustness = Double.NaN;
    private double lastShieldRobustness = Double.NaN;

    public AllSlowerShield(JavaHighwayEngine sourceEngine) {
        this.sourceEngine = sourceEngine;
        this.predictionTime = sourceEngine.config == null
                ? DEFAULT_PREDICTION_TIME
                : sourceEngine.config.getPredictionTime();
    }

    public AllSlowerShield(JavaHighwayEngine sourceEngine, int predictionTime) {
        this.sourceEngine = sourceEngine;
        this.predictionTime = predictionTime;
    }

    public boolean verifySafe() throws Exception {
        List<List<Vehicle>> trace = predictAllSlowerTrace();
        sequence = new FixedEvolutionSequence(toSampleSets(trace));

        int lastStep = lastPredictionStep(trace);
        int firstSecondLastStep = Math.min(sourceEngine.getFrequency() - 1, lastStep);
        DisTLFormula noCollision = new AlwaysDisTLFormula(
                new TargetDisTLFormula(this::resetCrashState, this::crashPenalty, CRASH_DISTANCE_THRESHOLD),
                0,
                lastStep
        );
        DisTLFormula safeFrontDistanceAtFirstSecond = new AlwaysDisTLFormula(
                new TargetDisTLFormula(this::stabilizeEgoAtFrontSafetyDistance, this::firstSecondFrontSafetyPenalty,
                        FIRST_SECOND_SAFETY_DISTANCE_THRESHOLD),
                firstSecondLastStep,
                firstSecondLastStep
        );
        DisTLFormula stableAtLastStep = new AlwaysDisTLFormula(
                new TargetDisTLFormula(this::stabilizeEgoAgainstFrontVehicle, this::frontVehicleStabilityPenalty,
                        STABILITY_DISTANCE_THRESHOLD),
                lastStep,
                lastStep
        );
        DisTLFormula shieldCondition = new ConjunctionDisTLFormula(
                noCollision,
                new ConjunctionDisTLFormula(safeFrontDistanceAtFirstSecond, stableAtLastStep)
        );

        DoubleSemanticsVisitor semantics = new DoubleSemanticsVisitor();
        lastCollisionRobustness = semantics.eval(noCollision).eval(EVOLUTION_SEQUENCE_SIZE, 0, sequence);
        lastFirstSecondSafetyRobustness = semantics.eval(safeFrontDistanceAtFirstSecond)
                .eval(EVOLUTION_SEQUENCE_SIZE, 0, sequence);
        lastStabilityRobustness = semantics.eval(stableAtLastStep).eval(EVOLUTION_SEQUENCE_SIZE, 0, sequence);
        lastShieldRobustness = semantics.eval(shieldCondition).eval(EVOLUTION_SEQUENCE_SIZE, 0, sequence);
        return lastShieldRobustness >= MIN_ACCEPTABLE_ROBUSTNESS;
    }

    public boolean hasCollision() {
        return sandboxEngine != null && sandboxEngine.hasCollision;
    }

    public boolean hasNpcCollision() {
        return sandboxEngine != null && sandboxEngine.hasNpcCollision;
    }

    public boolean hasNonNpcVehicleCollision() {
        return sandboxEngine != null && sandboxEngine.hasNonNpcVehicleCollision;
    }

    public SandboxJavaHighwayEngine getSandboxEngine() {
        return sandboxEngine;
    }

    public EvolutionSequence getSequence() {
        return sequence;
    }

    public double getLastCollisionRobustness() {
        return lastCollisionRobustness;
    }

    public double getLastFirstSecondSafetyRobustness() {
        return lastFirstSecondSafetyRobustness;
    }

    public double getLastStabilityRobustness() {
        return lastStabilityRobustness;
    }

    public double getLastShieldRobustness() {
        return lastShieldRobustness;
    }

    public String getUnsafeDiagnosis() {
        if (sequence == null) {
            return "Shield diagnosis unavailable: verifySafe() has not been called.";
        }
        int lastStep = Math.max(0, sequence.length() - 1);
        return sequence.get(lastStep).stream()
                .findFirst()
                .map(systemState -> diagnoseState(systemState.getDataState(), lastStep))
                .orElse("Shield diagnosis unavailable: no prediction state available.");
    }

    public String getFailedSafetyCriteriaCsv() {
        List<String> failed = new ArrayList<>();
        if (lastCollisionRobustness < MIN_ACCEPTABLE_ROBUSTNESS) {
            failed.add("collision");
        }
        if (lastFirstSecondSafetyRobustness < MIN_ACCEPTABLE_ROBUSTNESS) {
            failed.add("firstSecondSafety");
        }
        if (lastStabilityRobustness < MIN_ACCEPTABLE_ROBUSTNESS) {
            failed.add("stability");
        }
        return String.join(",", failed);
    }

    private List<List<Vehicle>> predictAllSlowerTrace() throws Exception {
        sandboxEngine = createSandboxEngine();
        applyAllSlower();

        int predictionSteps = Math.max(1, predictionTime * sandboxEngine.getFrequency());
        List<List<Vehicle>> trace = new ArrayList<>();
        trace.add(deepCopyVehicles(sandboxEngine.vehicles));
        for (int i = 1; i < predictionSteps; i++) {
            planSandboxActions();
            sandboxEngine.applyPhysics();
            sandboxEngine.checkCollision();
            trace.add(deepCopyVehicles(sandboxEngine.vehicles));
            sandboxEngine.timeElapsed += sandboxEngine.getDt();
            sandboxEngine.stepsTaken += 1;
        }
        return trace;
    }

    private List<SampleSet<SystemState>> toSampleSets(List<List<Vehicle>> trace) {
        List<SampleSet<SystemState>> sampleSets = new ArrayList<>();
        for (List<Vehicle> vehiclesAtStep : trace) {
            sampleSets.add(new SampleSet<>(List.of(new PerceivedSystemState(toDataState(vehiclesAtStep)))));
        }
        return sampleSets;
    }

    private DataState toDataState(List<Vehicle> vehiclesAtStep) {
        Map<Integer, Double> values = new HashMap<>();
        int vehicleCount = sourceEngine.vehicles.size();

        for (int i = 0; i < vehicleCount; i++) {
            Vehicle vehicle = i < vehiclesAtStep.size() ? vehiclesAtStep.get(i) : sourceEngine.vehicles.get(i);
            int offset = vehicleOffset(i);
            values.put(offset + VarTable.id.ordinal(), parseVehicleId(vehicle.id, i));
            values.put(offset + VarTable.politeness.ordinal(), vehicle.politeness);
            values.put(offset + VarTable.cooldownTimer.ordinal(), vehicle.cooldownTimer);
            values.put(offset + VarTable.target_lane_index.ordinal(), (double) vehicle.getTargetLaneIndex());
            values.put(offset + VarTable.lane_index.ordinal(), (double) vehicle.getLaneIndex());
            values.put(offset + VarTable.x.ordinal(), vehicle.x);
            values.put(offset + VarTable.y.ordinal(), vehicle.y);
            values.put(offset + VarTable.vx.ordinal(), vehicle.vx);
            values.put(offset + VarTable.vy.ordinal(), vehicle.vy);
            values.put(offset + VarTable.speed.ordinal(), vehicle.speed);
            values.put(offset + VarTable.heading.ordinal(), vehicle.heading);
            values.put(offset + VarTable.plannedAcceleration.ordinal(), vehicle.plannedAcceleration);
            values.put(offset + VarTable.plannedSteering.ordinal(), vehicle.plannedSteering);
            values.put(offset + VarTable.role.ordinal(), "EGO".equals(vehicle.role) ? 0.0 : 1.0);
            values.put(offset + VarTable.targetSpeed.ordinal(), vehicle.targetSpeed);
            values.put(offset + VarTable.idmCooldownTimer.ordinal(), 0.0);
            values.put(offset + VarTable.idmActionStepLength.ordinal(), 0.0);
            values.put(offset + VarTable.reactionDelay.ordinal(), 0.0);
        }

        values.put(crashedIndex(), hasCollision(vehiclesAtStep) ? 1.0 : 0.0);
        values.put(crashedIndex() + 1, -1.0);
        values.put(crashedIndex() + 2, -1.0);
        values.put(crashedIndex() + 3, -1.0);
        values.put(crashedIndex() + 4, isInitialChangeLane() ? 1.0 : 0.0);
        values.put(crashedIndex() + 5, -1.0);
        values.put(crashedIndex() + 6, 0.0);

        return new DataState(crashedIndex() + AUXILIARY_VAR_NUMS, index -> values.getOrDefault(index, Double.NaN));
    }

    private DataState resetCrashState(RandomGenerator rg, DataState state) {
        return state.apply(List.of(new DataStateUpdate(crashedIndex(), 0.0)));
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
        double safetyGap = calculateOneSecondWorstCaseSafetyGap(
                state.get(egoOffset + VarTable.vx.ordinal()),
                state.get(frontOffset + VarTable.vx.ordinal()),
                state.get(egoOffset + VarTable.plannedAcceleration.ordinal()),
                state.get(frontOffset + VarTable.plannedAcceleration.ordinal())
        );
        return state.apply(List.of(new DataStateUpdate(
                egoOffset + VarTable.x.ordinal(),
                state.get(frontOffset + VarTable.x.ordinal()) - VEHICLE_LENGTH - safetyGap
        )));
    }

    private DataState stabilizeEgoAgainstFrontVehicle(RandomGenerator rg, DataState state) {
        int egoIndex = getEgoVehicleIndex(state);
        if (egoIndex < 0) {
            return state;
        }

        List<Integer> referenceVehicles = getFinalStabilityReferenceVehicles(state, egoIndex);
        if (referenceVehicles.isEmpty()) {
            return state;
        }

        int egoOffset = vehicleOffset(egoIndex);
        double targetX = Double.POSITIVE_INFINITY;
        double targetVx = Double.POSITIVE_INFINITY;
        double targetSpeed = Double.POSITIVE_INFINITY;
        for (int vehicleIndex : referenceVehicles) {
            int offset = vehicleOffset(vehicleIndex);
            targetX = Math.min(targetX, state.get(offset + VarTable.x.ordinal()) - VEHICLE_LENGTH - MIN_STABLE_FRONT_GAP);
            targetVx = Math.min(targetVx, state.get(offset + VarTable.vx.ordinal()));
            targetSpeed = Math.min(targetSpeed, state.get(offset + VarTable.speed.ordinal()));
        }

        return state.apply(List.of(
                new DataStateUpdate(egoOffset + VarTable.x.ordinal(), targetX),
                new DataStateUpdate(egoOffset + VarTable.vx.ordinal(), targetVx),
                new DataStateUpdate(egoOffset + VarTable.speed.ordinal(), targetSpeed),
                new DataStateUpdate(egoOffset + VarTable.targetSpeed.ordinal(), targetSpeed)
        ));
    }

    private double crashPenalty(DataState state) {
        return state.get(crashedIndex()) > 0.0 ? 1.0 : 0.0;
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
        double frontGap = state.get(frontOffset + VarTable.x.ordinal())
                - state.get(egoOffset + VarTable.x.ordinal())
                - VEHICLE_LENGTH;
        double safetyGap = calculateOneSecondWorstCaseSafetyGap(
                state.get(egoOffset + VarTable.vx.ordinal()),
                state.get(frontOffset + VarTable.vx.ordinal()),
                state.get(egoOffset + VarTable.plannedAcceleration.ordinal()),
                state.get(frontOffset + VarTable.plannedAcceleration.ordinal())
        );
        if (safetyGap <= 0.0) {
            return 0.0;
        }
        return Math.min(1.0, Math.max(0.0, safetyGap - frontGap) / safetyGap);
    }

    private double frontVehicleStabilityPenalty(DataState state) {
        int egoIndex = getEgoVehicleIndex(state);
        if (egoIndex < 0) {
            return 0.0;
        }

        double totalPenalty = 0.0;
        for (int frontIndex : getFinalStabilityReferenceVehicles(state, egoIndex)) {
            totalPenalty += frontVehicleStabilityPenalty(state, egoIndex, frontIndex);
        }
        return totalPenalty;
    }

    private double frontVehicleStabilityPenalty(DataState state, int egoIndex, int frontIndex) {
        int egoOffset = vehicleOffset(egoIndex);
        int frontOffset = vehicleOffset(frontIndex);
        double frontGap = state.get(frontOffset + VarTable.x.ordinal())
                - state.get(egoOffset + VarTable.x.ordinal())
                - VEHICLE_LENGTH;
        double relativeSpeed = state.get(egoOffset + VarTable.vx.ordinal()) - state.get(frontOffset + VarTable.vx.ordinal());
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

        return aggressiveV3FrontStabilityPenalty(frontGap, closingSpeed);
    }

    private String diagnoseState(DataState state, int lastStep) {
        int egoIndex = getEgoVehicleIndex(state);
        StringBuilder diagnosis = new StringBuilder();
        diagnosis.append(String.format("Shield diagnosis at prediction step %d: crashed=%.0f",
                lastStep, state.get(crashedIndex())));
        diagnosis.append(String.format("%n  DisTL robustness: collision=%.3f firstSecondSafety=%.3f stability=%.3f shield=%.3f minAcceptable=%.3f",
                lastCollisionRobustness,
                lastFirstSecondSafetyRobustness,
                lastStabilityRobustness,
                lastShieldRobustness,
                MIN_ACCEPTABLE_ROBUSTNESS));

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

        for (int frontIndex : getFinalStabilityReferenceVehicles(state, egoIndex)) {
            int frontOffset = vehicleOffset(frontIndex);
            int lane = (int) state.get(frontOffset + VarTable.lane_index.ordinal());
            double frontGap = state.get(frontOffset + VarTable.x.ordinal())
                    - state.get(egoOffset + VarTable.x.ordinal())
                    - VEHICLE_LENGTH;
            double closingSpeed = Math.max(0.0,
                    state.get(egoOffset + VarTable.vx.ordinal()) - state.get(frontOffset + VarTable.vx.ordinal()));
            double penalty = frontVehicleStabilityPenalty(state, egoIndex, frontIndex);
            diagnosis.append(String.format("%n  front lane=%d vehicleId=%s vehicleIndex=%d gap=%.2f closingSpeed=%.2f penalty=%.3f",
                    lane,
                    vehicleId(state, frontIndex),
                    frontIndex,
                    frontGap,
                    closingSpeed,
                    penalty));
        }

        diagnosis.append(String.format("%n  totalFrontStabilityPenalty=%.3f threshold=%.3f",
                frontVehicleStabilityPenalty(state), STABILITY_DISTANCE_THRESHOLD));
        return diagnosis.toString();
    }

    private String vehicleId(DataState state, int vehicleIndex) {
        return String.valueOf((int) state.get(vehicleOffset(vehicleIndex) + VarTable.id.ordinal()));
    }

    private double aggressiveV3FrontStabilityPenalty(double frontGap, double closingSpeed) {
        if (closingSpeed <= 0.0) {
            return 0.0;
        }
        if (frontGap < MIN_STABLE_FRONT_GAP) {
            return Math.min(1.0, Math.max(0.0, closingSpeed - MAX_STABLE_RELATIVE_SPEED) / MAX_STABLE_RELATIVE_SPEED);
        }
        double ttc = frontGap / closingSpeed;
        if (ttc > AGGRESSIVE_V3_TTC_THRESHOLD) {
            return 0.0;
        }
        return Math.min(1.0, (AGGRESSIVE_V3_TTC_THRESHOLD - ttc) / AGGRESSIVE_V3_TTC_THRESHOLD);
    }

    private double calculateOneSecondWorstCaseSafetyGap(double egoVx, double frontVx, double egoAcceleration,
                                                       double frontAcceleration) {
        double egoSpeed = Math.max(0.0, egoVx);
        double frontSpeed = Math.max(0.0, frontVx);
        double frontWorstAcceleration = frontAcceleration - MIN_FRONT_ACCELERATION_UNCERTAINTY;
        double relativeClosingDistance = (egoSpeed - frontSpeed) * FIRST_SECOND_LOOKAHEAD_TIME
                + 0.5 * (egoAcceleration - frontWorstAcceleration)
                * FIRST_SECOND_LOOKAHEAD_TIME * FIRST_SECOND_LOOKAHEAD_TIME;
        return FIRST_SECOND_MIN_FRONT_GAP + Math.max(0.0, relativeClosingDistance);
    }

    private int getEgoVehicleIndex(DataState state) {
        for (int i = 0; i < sourceEngine.vehicles.size(); i++) {
            if (state.get(vehicleOffset(i) + VarTable.role.ordinal()) == 0.0) {
                return i;
            }
        }
        return -1;
    }

    private List<Integer> getVehiclesInAdjacentLanes(DataState state, int egoIndex) {
        List<Integer> adjacentVehicles = new ArrayList<>();
        int egoOffset = vehicleOffset(egoIndex);
        int egoLane = (int) state.get(egoOffset + VarTable.lane_index.ordinal());
        for (int i = 0; i < sourceEngine.vehicles.size(); i++) {
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

    private List<Integer> getFinalStabilityReferenceVehicles(DataState state, int egoIndex) {
        int egoLane = (int) state.get(vehicleOffset(egoIndex) + VarTable.lane_index.ordinal());
        double egoX = state.get(vehicleOffset(egoIndex) + VarTable.x.ordinal());
        int closestReferenceIndex = -1;
        double closestReferenceX = Double.POSITIVE_INFINITY;

        int frontIndex = getFrontVehicleIndexInLane(state, egoIndex, egoLane);
        if (frontIndex >= 0) {
            closestReferenceIndex = frontIndex;
            closestReferenceX = state.get(vehicleOffset(frontIndex) + VarTable.x.ordinal());
        }

        for (int i = 0; i < sourceEngine.vehicles.size(); i++) {
            if (i == egoIndex) {
                continue;
            }
            int offset = vehicleOffset(i);
            int lane = (int) state.get(offset + VarTable.lane_index.ordinal());
            double x = state.get(offset + VarTable.x.ordinal());
            if (lane != egoLane
                    && x > egoX
                    && x < closestReferenceX
                    && hasHistoricalCutInIntentTowardEgoLane(state, i, egoLane)) {
                closestReferenceIndex = i;
                closestReferenceX = x;
            }
        }

        List<Integer> vehicles = new ArrayList<>();
        if (closestReferenceIndex >= 0) {
            vehicles.add(closestReferenceIndex);
        }
        return vehicles;
    }

    private boolean hasHistoricalCutInIntentTowardEgoLane(DataState state, int vehicleIndex, int egoLane) {
        int historyIndex = crashedIndex() + AUXILIARY_VAR_NUMS + vehicleIndex;
        if (historyIndex < state.size()) {
            return state.get(historyIndex) > 0.0;
        }
        int offset = vehicleOffset(vehicleIndex);
        int targetLane = (int) state.get(offset + VarTable.target_lane_index.ordinal());
        return targetLane == egoLane;
    }

    private int getFrontVehicleIndexInLane(DataState state, int egoIndex, int targetLane) {
        if (egoIndex < 0) {
            return -1;
        }

        int egoOffset = vehicleOffset(egoIndex);
        double egoX = state.get(egoOffset + VarTable.x.ordinal());
        int frontIndex = -1;
        double closestFrontX = Double.POSITIVE_INFINITY;
        for (int i = 0; i < sourceEngine.vehicles.size(); i++) {
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
        return sourceEngine.vehicles.size() * VarTable.values().length;
    }

    private int lastPredictionStep(List<List<Vehicle>> trace) {
        return Math.max(0, trace.size() - 1);
    }

    private double parseVehicleId(String id, int fallback) {
        try {
            return Double.parseDouble(id);
        } catch (Exception ignored) {
            return fallback;
        }
    }

    private boolean isInitialChangeLane() {
        for (Vehicle vehicle : sourceEngine.vehicles) {
            if ("EGO".equals(vehicle.role)) {
                return vehicle.getLaneIndex() != vehicle.getTargetLaneIndex();
            }
        }
        return false;
    }

    private boolean hasCollision(List<Vehicle> vehiclesAtStep) {
        for (int i = 0; i < vehiclesAtStep.size(); i++) {
            for (int j = i + 1; j < vehiclesAtStep.size(); j++) {
                Vehicle first = vehiclesAtStep.get(i);
                Vehicle second = vehiclesAtStep.get(j);
                boolean overlapX = Math.abs(first.x - second.x) < (first.LENGTH / 2.0 + second.LENGTH / 2.0);
                boolean overlapY = Math.abs(first.y - second.y) < (first.WIDTH / 2.0 + second.WIDTH / 2.0);
                if (overlapX && overlapY) {
                    return true;
                }
            }
        }
        return false;
    }

    private SandboxJavaHighwayEngine createSandboxEngine() {
        SandboxJavaHighwayEngine sandbox = new SandboxJavaHighwayEngine();
        sandbox.setFrequency(sourceEngine.getFrequency());
        sandbox.config = sourceEngine.config;
        sandbox.numLanes = sourceEngine.numLanes;
        sandbox.timeElapsed = sourceEngine.timeElapsed;
        sandbox.stepsTaken = sourceEngine.stepsTaken;
        sandbox.vehicles = deepCopyVehicles(sourceEngine.vehicles);
        for (Vehicle vehicle : sandbox.vehicles) {
            vehicle.setEngine(sandbox);
        }
        return sandbox;
    }

    private List<Vehicle> deepCopyVehicles(List<Vehicle> vehicles) {
        List<Vehicle> copies = new ArrayList<>();
        for (Vehicle vehicle : vehicles) {
            Vehicle copy = vehicle instanceof NonNpcVehicle ? new ShieldNonNpcVehicle() : new Vehicle();
            copyVehicleState(vehicle, copy);
            copies.add(copy);
        }
        return copies;
    }

    private void copyVehicleState(Vehicle source, Vehicle copy) {
        copy.TAU_ACC = source.TAU_ACC;
        copy.TAU_HEADING = source.TAU_HEADING;
        copy.TAU_LATERAL = source.TAU_LATERAL;
        copy.TAU_PURSUIT = source.TAU_PURSUIT;
        copy.KP_A = source.KP_A;
        copy.KP_HEADING = source.KP_HEADING;
        copy.KP_LATERAL = source.KP_LATERAL;
        copy.MAX_STEERING_ANGLE = source.MAX_STEERING_ANGLE;
        copy.DELTA_SPEED = source.DELTA_SPEED;
        copy.possible_lanes = source.possible_lanes.clone();
        copy.karma_a_new = source.karma_a_new;
        copy.mobil = source.mobil;
        copy.targetSpeed = source.targetSpeed;
        copy.id = source.id;
        copy.politeness = source.politeness;
        copy.cooldownTimer = source.cooldownTimer;
        copy.setTargetLaneIndex(source.getTargetLaneIndex());
        copy.setLaneIndex(source.getLaneIndex());
        copy.role = source.role;
        copy.x = source.x;
        copy.y = source.y;
        copy.vx = source.vx;
        copy.vy = source.vy;
        copy.speed = source.speed;
        copy.previousSecondSpeed = source.previousSecondSpeed;
        copy.heading = source.heading;
        copy.plannedAcceleration = source.plannedAcceleration;
        copy.plannedSteering = source.plannedSteering;
        if (isFixPredictionEnabled() && !(copy instanceof NonNpcVehicle)) {
            copy.targetSpeed = getRandomFixedPredictionTargetSpeed(copy);
        }
    }

    private void applyAllSlower() {
        for (Vehicle vehicle : sandboxEngine.vehicles) {
            if (vehicle instanceof NonNpcVehicle) {
                vehicle.targetSpeed = Math.max(0.0, vehicle.targetSpeed - 5);
            }
        }
    }

    private boolean isFixPredictionEnabled() {
        return sourceEngine.config != null && sourceEngine.config.isFixPrediction();
    }

    private double getRandomFixedPredictionTargetSpeed(Vehicle vehicle) {
        double minTargetSpeed = Math.max(0.0, vehicle.speed - FIXED_PREDICTION_RANDOM_TARGET_DELTA);
        double maxTargetSpeed = vehicle.speed + FIXED_PREDICTION_RANDOM_TARGET_DELTA;
        return minTargetSpeed + Math.random() * (maxTargetSpeed - minTargetSpeed);
    }

    private void planSandboxActions() throws Exception {
        for (Vehicle vehicle : sandboxEngine.vehicles) {
            if (vehicle instanceof NonNpcVehicle) {
                vehicle.plannedAcceleration = JavaHighwayEngineUtils.computeIdmAcceleration(vehicle, sandboxEngine.vehicles);
                vehicle.plannedSteering = JavaHighwayEngineUtils.computeSteering(vehicle);
            } else {
                vehicle.planAction(sandboxEngine.vehicles);
            }
        }
    }

    private static class ShieldNonNpcVehicle extends Vehicle implements NonNpcVehicle {
        @Override
        public void planAction() {
        }
    }

    private static class FixedEvolutionSequence extends EvolutionSequence {
        FixedEvolutionSequence(List<SampleSet<SystemState>> sequence) {
            super(null, new DefaultRandomGenerator(), sequence);
        }
    }
}
