package Scenarios.Engine;

import com.google.gson.Gson;
import com.google.gson.annotations.SerializedName;
import Scenarios.StarkScenarioRunner;
import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.SampleSet;
import it.unicam.quasylab.jspear.SystemState;
import it.unicam.quasylab.jspear.ds.DataState;
import nl.tue.Monitoring.PerceivedSystemState;
import py4j.GatewayServer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class PythonMomentumShieldApp {
    private static final double DEFAULT_DT = 1.0 / 15.0;
    private static final double LANE_WIDTH = 4.0;

    private final Gson gson = new Gson();

    private double radiusMeters = 200.0;
    private boolean randomizeNpcPoliteness = false;
    private boolean randomizeNpcCooldownTimer = false;
    private boolean randomizeNpcTargetSpeed = false;
    private boolean checkFirstSecondSafety = StarkShieldApp.DEFAULT_CHECK_FIRST_SECOND_SAFETY;
    private boolean checkChangeLaneToRearVehicleThreat = false;
    private boolean aggressiveFinalStability = false;
    private StarkShieldApp.FinalStabilityPenaltyMode finalStabilityPenaltyMode =
            StarkShieldApp.DEFAULT_FINAL_STABILITY_PENALTY_MODE;
    private String aiProfile = HighwayAiClient.getConfiguredAiProfile();
    private double dt = DEFAULT_DT;
    private int numLanes = 3;
    private int predictFutureSeconds = 3;
    private boolean printDiagnostics = false;
    private String lastFailedSafetyCriteria = "";

    public boolean verifySafe(String action, String traceJson) {
        return verifySafetyWithTrace(action, traceJson);
    }

    public boolean verifySafetyWithControllerDrivenPrediction(String action, String stateJson) {
        try {
            HighwayEnvVehicleState[] states = gson.fromJson(stateJson, HighwayEnvVehicleState[].class);
            if (states == null || states.length == 0) {
                throw new IllegalArgumentException("stateJson must contain at least one current state");
            }

            List<Vehicle> vehicles = toShieldVehicles(states);
            vehicles = keepOnlyLongitudinalRadius(List.of(vehicles), radiusMeters).get(0);

            HighwayEngine shieldEngine = new HighwayEngine(dt, true, true, vehicles);
            shieldEngine.numLanes = inferNumLanes(vehicles);
            StarkShieldApp.setCheckFirstSecondSafety(checkFirstSecondSafety);

            ControlledVehicle ego = findControlledEgo(vehicles);
            ego.applyAiAction(toAiDecision(action));

            ControllerDrivenStarkShieldApp shield = new ControllerDrivenStarkShieldApp(
                    shieldEngine,
                    vehicles,
                    predictFutureSeconds,
                    Double.MAX_VALUE,
                    randomizeNpcCooldownTimer || randomizeNpcTargetSpeed,
                    checkChangeLaneToRearVehicleThreat,
                    StarkShieldApp.DEFAULT_READ_SHIELD_IDM_COOLDOWN_TIMER,
                    StarkShieldApp.DEFAULT_FIX_PREDICTION,
                    aggressiveFinalStability,
                    finalStabilityPenaltyMode
            );

            boolean safe = shield.verifySafe();
            lastFailedSafetyCriteria = safe ? "" : shield.getFailedSafetyCriteriaCsv();
            if (printDiagnostics || !safe) {
                System.out.printf("PythonMomentumShield(controller-driven): action=%s aiProfile=%s safe=%s radius=%.1f predictionSeconds=%d%n",
                        action, aiProfile, safe, radiusMeters, predictFutureSeconds);
                System.out.println(shield.getUnsafeDiagnosis());
            }
            return safe;
        } catch (Exception exception) {
            lastFailedSafetyCriteria = "shieldException";
            System.err.println("PythonMomentumShield controller-driven prediction failed: " + exception.getMessage());
            exception.printStackTrace();
            return false;
        }
    }

    public boolean verifySafetyWithTrace(String action, String traceJson) {
        try {
            HighwayEnvVehicleState[][] traceStates = gson.fromJson(traceJson, HighwayEnvVehicleState[][].class);
            if (traceStates == null || traceStates.length == 0) {
                throw new IllegalArgumentException("traceJson must contain at least one rollout state");
            }

            List<List<Vehicle>> trace = new ArrayList<>();
            for (HighwayEnvVehicleState[] state : traceStates) {
                trace.add(toShieldVehicles(state));
            }
            trace = keepOnlyLongitudinalRadius(trace, radiusMeters);

            List<Vehicle> initialVehicles = trace.get(0);
            HighwayEngine shieldEngine = new HighwayEngine(dt, true, true, initialVehicles);
            shieldEngine.numLanes = inferNumLanes(initialVehicles);
            StarkShieldApp.setCheckFirstSecondSafety(checkFirstSecondSafety);

            FixedTraceShield shield = new FixedTraceShield(
                    shieldEngine,
                    trace,
                    checkChangeLaneToRearVehicleThreat,
                    aggressiveFinalStability,
                    finalStabilityPenaltyMode
            );

            boolean safe = shield.verifySafe();
            lastFailedSafetyCriteria = safe ? "" : shield.getFailedSafetyCriteriaCsv();
            if (printDiagnostics || !safe) {
                System.out.printf("PythonMomentumShield: action=%s aiProfile=%s safe=%s radius=%.1f%n",
                        action, aiProfile, safe, radiusMeters);
                System.out.println(shield.getUnsafeDiagnosis());
            }
            return safe;
        } catch (Exception exception) {
            lastFailedSafetyCriteria = "shieldException";
            System.err.println("PythonMomentumShield failed: " + exception.getMessage());
            exception.printStackTrace();
            return false;
        }
    }

    public boolean verifySafetyWithSampleTraces(String action, String sampleTracesJson) {
        try {
            HighwayEnvVehicleState[][][] sampleTraceStates = gson.fromJson(sampleTracesJson, HighwayEnvVehicleState[][][].class);
            if (sampleTraceStates == null || sampleTraceStates.length == 0 || sampleTraceStates[0].length == 0) {
                throw new IllegalArgumentException("sampleTracesJson must contain at least one sample trace");
            }

            List<List<List<Vehicle>>> sampleTraces = new ArrayList<>();
            for (HighwayEnvVehicleState[][] sampleTraceState : sampleTraceStates) {
                List<List<Vehicle>> trace = new ArrayList<>();
                for (HighwayEnvVehicleState[] state : sampleTraceState) {
                    trace.add(toShieldVehicles(state));
                }
                sampleTraces.add(keepOnlyLongitudinalRadius(trace, radiusMeters));
            }

            List<Vehicle> initialVehicles = sampleTraces.get(0).get(0);
            HighwayEngine shieldEngine = new HighwayEngine(dt, true, true, initialVehicles);
            shieldEngine.numLanes = inferNumLanes(initialVehicles);
            StarkShieldApp.setCheckFirstSecondSafety(checkFirstSecondSafety);

            FixedTraceShield shield = new FixedTraceShield(
                    shieldEngine,
                    sampleTraces,
                    checkChangeLaneToRearVehicleThreat,
                    aggressiveFinalStability,
                    finalStabilityPenaltyMode,
                    true
            );

            boolean safe = shield.verifySafe();
            lastFailedSafetyCriteria = safe ? "" : shield.getFailedSafetyCriteriaCsv();
            if (printDiagnostics || !safe) {
                System.out.printf("PythonMomentumShield: action=%s aiProfile=%s samples=%d safe=%s radius=%.1f%n",
                        action, aiProfile, sampleTraces.size(), safe, radiusMeters);
                System.out.println(shield.getUnsafeDiagnosis());
            }
            return safe;
        } catch (Exception exception) {
            lastFailedSafetyCriteria = "shieldException";
            System.err.println("PythonMomentumShield failed: " + exception.getMessage());
            exception.printStackTrace();
            return false;
        }
    }

    public String getLastFailedSafetyCriteria() {
        return lastFailedSafetyCriteria;
    }

    public void setRadiusMeters(double radiusMeters) {
        this.radiusMeters = Math.max(0.0, radiusMeters);
    }

    public double getRadiusMeters() {
        return radiusMeters;
    }

    public void setRandomizeNpcPoliteness(boolean randomizeNpcPoliteness) {
        this.randomizeNpcPoliteness = randomizeNpcPoliteness;
    }

    public boolean getRandomizeNpcPoliteness() {
        return randomizeNpcPoliteness;
    }

    public void setRandomizeNpcCooldownTimer(boolean randomizeNpcCooldownTimer) {
        this.randomizeNpcCooldownTimer = randomizeNpcCooldownTimer;
    }

    public boolean getRandomizeNpcCooldownTimer() {
        return randomizeNpcCooldownTimer;
    }

    public void setRandomizeNpcTargetSpeed(boolean randomizeNpcTargetSpeed) {
        this.randomizeNpcTargetSpeed = randomizeNpcTargetSpeed;
    }

    public boolean getRandomizeNpcTargetSpeed() {
        return randomizeNpcTargetSpeed;
    }

    public void setAiProfile(String aiProfile) {
        this.aiProfile = aiProfile == null || aiProfile.isBlank() ? "base" : aiProfile.trim();
    }

    public String getAiProfile() {
        return aiProfile;
    }

    public void setCheckChangeLaneToRearVehicleThreat(boolean checkChangeLaneToRearVehicleThreat) {
        this.checkChangeLaneToRearVehicleThreat = checkChangeLaneToRearVehicleThreat;
    }

    public void setCheckFirstSecondSafety(boolean checkFirstSecondSafety) {
        this.checkFirstSecondSafety = checkFirstSecondSafety;
    }

    public void setAggressiveFinalStability(boolean aggressiveFinalStability) {
        this.aggressiveFinalStability = aggressiveFinalStability;
    }

    public void setAggressiveFinalStabilityV2(boolean enabled) {
        if (enabled) {
            this.aggressiveFinalStability = true;
            this.finalStabilityPenaltyMode = StarkShieldApp.FinalStabilityPenaltyMode.AGGRESSIVE_V2;
        } else {
            this.finalStabilityPenaltyMode = StarkShieldApp.DEFAULT_FINAL_STABILITY_PENALTY_MODE;
        }
    }

    public void setFinalStabilityPenaltyMode(String mode) {
        if (mode == null || mode.isBlank()) {
            this.finalStabilityPenaltyMode = StarkShieldApp.DEFAULT_FINAL_STABILITY_PENALTY_MODE;
            return;
        }
        this.finalStabilityPenaltyMode = StarkShieldApp.FinalStabilityPenaltyMode.valueOf(mode.trim().toUpperCase());
    }

    public void setFinalStabilityMode(String mode) {
        if (mode == null || mode.isBlank()) {
            this.aggressiveFinalStability = StarkShieldApp.DEFAULT_AGGRESSIVE_FINAL_STABILITY;
            this.finalStabilityPenaltyMode = StarkShieldApp.DEFAULT_FINAL_STABILITY_PENALTY_MODE;
            return;
        }
        StarkScenarioRunner.FinalStabilityMode resolvedMode =
                StarkScenarioRunner.FinalStabilityMode.valueOf(mode.trim().toUpperCase());
        this.aggressiveFinalStability = resolvedMode.aggressiveFinalStability();
        this.finalStabilityPenaltyMode = resolvedMode.penaltyMode();
    }

    public void setAggressiveV2MaxStableRelativeSpeed(double threshold) {
        StarkShieldApp.setAggressiveV2MaxStableRelativeSpeed(threshold);
    }

    public void setDt(double dt) {
        if (dt <= 0.0) {
            throw new IllegalArgumentException("dt must be positive");
        }
        this.dt = dt;
    }

    public void setNumLanes(int numLanes) {
        this.numLanes = Math.max(1, numLanes);
    }

    public void setPredictionSeconds(double predictionSeconds) {
        this.predictFutureSeconds = Math.max(1, (int) Math.round(predictionSeconds));
    }

    public void setPrintDiagnostics(boolean printDiagnostics) {
        this.printDiagnostics = printDiagnostics;
    }

    private ControlledVehicle findControlledEgo(List<Vehicle> vehicles) {
        Vehicle ego = findEgo(vehicles);
        if (ego instanceof ControlledVehicle controlledVehicle) {
            return controlledVehicle;
        }
        throw new IllegalArgumentException("controller-driven prediction requires the EGO vehicle to be controlled");
    }

    private HighwayAiClient.AiDecision toAiDecision(String action) {
        int actionIndex;
        try {
            actionIndex = Integer.parseInt(action == null ? "1" : action.trim());
        } catch (Exception ignored) {
            actionIndex = 1;
        }
        HighwayAiClient.AiDecision decision = new HighwayAiClient.AiDecision();
        decision.action = actionIndex;
        decision.action_name = switch (actionIndex) {
            case 0 -> "LANE_LEFT";
            case 2 -> "LANE_RIGHT";
            case 3 -> "FASTER";
            case 4 -> "SLOWER";
            default -> "IDLE";
        };
        return decision;
    }

    private List<Vehicle> toShieldVehicles(HighwayEnvVehicleState[] states) {
        List<Vehicle> vehicles = new ArrayList<>();
        for (int i = 0; i < states.length; i++) {
            HighwayEnvVehicleState state = states[i];
            Vehicle vehicle = isEgo(state) ? new ControlledVehicle() : new Vehicle();

            vehicle.id = state.id == null || state.id.isBlank() ? String.valueOf(i) : state.id;
            vehicle.role = isEgo(state) ? "EGO" : "NPC";
            vehicle.politeness = state.valueOrDefault(state.politeness, 0.0);
            vehicle.cooldownTimer = state.valueOrDefault(state.cooldownTimer, 0.0);

            int lane = state.getLane();
            vehicle.setLaneIndex(lane);
            vehicle.setTargetLaneIndex(state.hasTargetLane() ? state.getTargetLane() : lane);

            vehicle.x = state.getX();
            vehicle.y = state.hasY() ? state.y : lane * LANE_WIDTH;
            vehicle.vx = state.hasVx() ? state.vx : state.getSpeed();
            vehicle.vy = state.hasVy() ? state.vy : 0.0;
            vehicle.speed = state.getSpeed();
            vehicle.heading = state.valueOrDefault(state.heading, 0.0);
            vehicle.plannedAcceleration = state.valueOrDefault(state.acceleration, 0.0);
            vehicle.plannedSteering = state.valueOrDefault(state.steering, 0.0);
            vehicle.targetSpeed = state.getTargetSpeed();

            vehicles.add(vehicle);
        }

        boolean hasEgo = vehicles.stream().anyMatch(vehicle -> "EGO".equals(vehicle.role));
        if (!hasEgo) {
            throw new IllegalArgumentException("trace must include one vehicle with role='EGO'");
        }
        return vehicles;
    }

    private List<List<Vehicle>> keepOnlyLongitudinalRadius(List<List<Vehicle>> trace, double radiusMeters) {
        if (trace.isEmpty()) {
            return trace;
        }

        List<String> keptIds = new ArrayList<>();
        Vehicle initialEgo = findEgo(trace.get(0));
        for (Vehicle vehicle : trace.get(0)) {
            if (vehicle == initialEgo || Math.abs(vehicle.x - initialEgo.x) <= radiusMeters) {
                keptIds.add(vehicle.id);
            }
        }

        List<List<Vehicle>> filteredTrace = new ArrayList<>();
        for (List<Vehicle> step : trace) {
            List<Vehicle> filteredStep = new ArrayList<>();
            for (String id : keptIds) {
                Vehicle vehicle = findById(step, id);
                if (vehicle != null) {
                    filteredStep.add(vehicle);
                }
            }
            filteredTrace.add(filteredStep);
        }
        return filteredTrace;
    }

    private Vehicle findEgo(List<Vehicle> vehicles) {
        return vehicles.stream()
                .filter(vehicle -> "EGO".equals(vehicle.role))
                .findFirst()
                .orElseThrow(() -> new IllegalArgumentException("trace must include one ego vehicle"));
    }

    private Vehicle findById(List<Vehicle> vehicles, String id) {
        for (Vehicle vehicle : vehicles) {
            if (vehicle.id.equals(id)) {
                return vehicle;
            }
        }
        return null;
    }

    private boolean isEgo(HighwayEnvVehicleState state) {
        return state.role != null && state.role.trim().equalsIgnoreCase("EGO");
    }

    private int inferNumLanes(List<Vehicle> vehicles) {
        int maxLane = numLanes - 1;
        for (Vehicle vehicle : vehicles) {
            maxLane = Math.max(maxLane, vehicle.getLaneIndex());
            maxLane = Math.max(maxLane, vehicle.getTargetLaneIndex());
        }
        return Math.max(1, maxLane + 1);
    }

    public static void main(String[] args) {
        PythonMomentumShieldApp app = new PythonMomentumShieldApp();
        GatewayServer server = new GatewayServer(app);
        server.start();
        System.out.println("PythonMomentumShield GatewayServer started.");
    }

    private static class FixedTraceShield extends StarkShieldApp {
        FixedTraceShield(HighwayEngine engine, List<List<Vehicle>> trace,
                         boolean checkChangeLaneToRearVehicleThreat,
                         boolean aggressiveFinalStability,
                         StarkShieldApp.FinalStabilityPenaltyMode finalStabilityPenaltyMode) {
            super(engine, trace.get(0), Math.max(1, trace.size()),
                    Double.MAX_VALUE, false, checkChangeLaneToRearVehicleThreat,
                    StarkShieldApp.DEFAULT_READ_SHIELD_IDM_COOLDOWN_TIMER,
                    StarkShieldApp.DEFAULT_FIX_PREDICTION,
                    aggressiveFinalStability,
                    finalStabilityPenaltyMode);
            this.predictionStepCountOverride = trace.size();
            this.sequence = new FixedEvolutionSequence(toSampleSets(trace));
        }

        FixedTraceShield(HighwayEngine engine, List<List<List<Vehicle>>> sampleTraces,
                         boolean checkChangeLaneToRearVehicleThreat,
                         boolean aggressiveFinalStability,
                         StarkShieldApp.FinalStabilityPenaltyMode finalStabilityPenaltyMode,
                         boolean sampleMajor) {
            super(engine, sampleTraces.get(0).get(0), 1,
                    Double.MAX_VALUE, false, checkChangeLaneToRearVehicleThreat,
                    StarkShieldApp.DEFAULT_READ_SHIELD_IDM_COOLDOWN_TIMER,
                    StarkShieldApp.DEFAULT_FIX_PREDICTION,
                    aggressiveFinalStability,
                    finalStabilityPenaltyMode);
            this.predictionStepCountOverride = sampleTraces.get(0).size();
            this.sequence = new FixedEvolutionSequence(toSampleSetsFromSampleTraces(sampleTraces));
        }

        private List<SampleSet<SystemState>> toSampleSets(List<List<Vehicle>> trace) {
            List<SampleSet<SystemState>> sampleSets = new ArrayList<>();
            for (List<Vehicle> vehiclesAtStep : trace) {
                sampleSets.add(new SampleSet<>(List.of(new PerceivedSystemState(toDataState(vehiclesAtStep)))));
            }
            return sampleSets;
        }

        private List<SampleSet<SystemState>> toSampleSetsFromSampleTraces(List<List<List<Vehicle>>> sampleTraces) {
            int stepCount = sampleTraces.get(0).size();
            List<SampleSet<SystemState>> sampleSets = new ArrayList<>();
            for (int step = 0; step < stepCount; step++) {
                List<SystemState> samplesAtStep = new ArrayList<>();
                for (List<List<Vehicle>> sampleTrace : sampleTraces) {
                    if (sampleTrace.size() != stepCount) {
                        throw new IllegalArgumentException("All sample traces must have the same length");
                    }
                    samplesAtStep.add(new PerceivedSystemState(toDataState(sampleTrace.get(step))));
                }
                sampleSets.add(new SampleSet<>(samplesAtStep));
            }
            return sampleSets;
        }

        private DataState toDataState(List<Vehicle> vehiclesAtStep) {
            Map<Integer, Double> values = new HashMap<>();
            int vehicleCount = vehicles.size();

            for (int i = 0; i < vehicleCount; i++) {
                Vehicle vehicle = i < vehiclesAtStep.size() ? vehiclesAtStep.get(i) : vehicles.get(i);
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
            values.put(crashedIndex() + 5, (double) initialTargetRearIndex());
            values.put(crashedIndex() + 6, 0.0);

            return new DataState(crashedIndex() + auxilaryVarNums, index -> values.getOrDefault(index, Double.NaN));
        }

        private double parseVehicleId(String id, int fallback) {
            try {
                return Double.parseDouble(id);
            } catch (Exception ignored) {
                return fallback;
            }
        }

        private boolean isInitialChangeLane() {
            for (Vehicle vehicle : vehicles) {
                if ("EGO".equals(vehicle.role)) {
                    return vehicle.getLaneIndex() != vehicle.getTargetLaneIndex();
                }
            }
            return false;
        }

        private int initialTargetRearIndex() {
            int egoIndex = -1;
            for (int i = 0; i < vehicles.size(); i++) {
                if ("EGO".equals(vehicles.get(i).role)) {
                    egoIndex = i;
                    break;
                }
            }
            if (egoIndex < 0 || !isInitialChangeLane()) {
                return -1;
            }
            Vehicle ego = vehicles.get(egoIndex);
            int targetLane = ego.getTargetLaneIndex();
            int rearIndex = -1;
            double closestRearX = -Double.MAX_VALUE;
            for (int i = 0; i < vehicles.size(); i++) {
                if (i == egoIndex) {
                    continue;
                }
                Vehicle candidate = vehicles.get(i);
                if (candidate.getLaneIndex() == targetLane && candidate.x < ego.x && candidate.x > closestRearX) {
                    rearIndex = i;
                    closestRearX = candidate.x;
                }
            }
            return rearIndex;
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
    }

    private static class FixedEvolutionSequence extends EvolutionSequence {
        FixedEvolutionSequence(List<SampleSet<SystemState>> sequence) {
            super(null, new DefaultRandomGenerator(), sequence);
        }
    }

    private static class HighwayEnvVehicleState {
        String id;
        String role;
        Double x;
        Double y;
        Double dist;
        Integer lane;

        @SerializedName("target_lane")
        Integer targetLane;

        Double vx;
        Double vy;
        Double speed;
        Double heading;
        Double acceleration;
        Double steering;
        Double politeness;

        @SerializedName("cooldown_timer")
        Double cooldownTimer;

        @SerializedName("target_speed")
        Double targetSpeedSnake;

        Double targetSpeed;

        double getX() {
            return valueOrDefault(x, valueOrDefault(dist, 0.0));
        }

        boolean hasY() {
            return y != null;
        }

        boolean hasVx() {
            return vx != null;
        }

        boolean hasVy() {
            return vy != null;
        }

        int getLane() {
            return lane == null ? 0 : lane;
        }

        boolean hasTargetLane() {
            return targetLane != null;
        }

        int getTargetLane() {
            return targetLane == null ? getLane() : targetLane;
        }

        double getSpeed() {
            if (speed != null) {
                return speed;
            }
            if (vx != null) {
                return Math.abs(vx);
            }
            return 0.0;
        }

        double getTargetSpeed() {
            if (targetSpeedSnake != null) {
                return targetSpeedSnake;
            }
            if (targetSpeed != null) {
                return targetSpeed;
            }
            return getSpeed();
        }

        double valueOrDefault(Double value, double defaultValue) {
            return value == null || value.isNaN() ? defaultValue : value;
        }
    }
}
