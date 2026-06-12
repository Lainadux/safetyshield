package Scenarios.Engine;

import com.google.gson.Gson;
import com.google.gson.annotations.SerializedName;
import py4j.GatewayServer;

import java.util.ArrayList;
import java.util.List;

/**
 * Py4J entry point for using the STARK shield while the real training world is
 * executed by highway-env.
 *
 * Python sends the current highway-env state and the AI proposed action. This
 * class builds a temporary shield state, applies the proposed action to the ego
 * vehicle, and delegates the unchanged safety logic to StarkShieldApp.
 */
public class  StarkForTrainingAI {
    private static final double DEFAULT_DT = 1.0 / 15.0;
    private static final double LANE_WIDTH = 4.0;

    private final Gson gson = new Gson();

    private int predictFutureSeconds = 3;
    private double shieldEgoRangeMeters = StarkShieldApp.DEFAULT_SHIELD_EGO_RANGE_METERS;
    private boolean randomizeHiddenTargetAndCooldown = StarkShieldApp.DEFAULT_RANDOMIZE_HIDDEN_TARGET_AND_COOLDOWN;
    private boolean checkChangeLaneToRearVehicleThreat = false;
    private boolean readShieldIdmCooldownTimer = StarkShieldApp.DEFAULT_READ_SHIELD_IDM_COOLDOWN_TIMER;
    private double dt = DEFAULT_DT;
    private int numLanes = 3;
    private boolean printDiagnostics = false;

    public boolean verifySafety(String action, String vehiclesJson) {
        System.err.println("verifySafety(action, vehiclesJson) is disabled for training: pass a highway-env rollout trace with verifySafetyWithTrace(action, traceJson).");
        return false;
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

            List<Vehicle> vehicles = trace.get(0);
            HighwayEngine shieldEngine = new HighwayEngine(dt, true, true, vehicles);
            shieldEngine.numLanes = inferNumLanes(vehicles);

            TraceBasedStarkShieldApp shield = new TraceBasedStarkShieldApp(shieldEngine, shieldEngine.vehicles, trace);

            boolean safe = shield.verifySafe();
            if (printDiagnostics || !safe) {
                System.out.printf("STARK training shield: action=%s safe=%s%n", action, safe);
                System.out.println(shield.getUnsafeDiagnosis());
            }
            return safe;
        } catch (Exception exception) {
            System.err.println("STARK training shield failed: " + exception.getMessage());
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
                List<List<Vehicle>> sampleTrace = new ArrayList<>();
                for (HighwayEnvVehicleState[] state : sampleTraceState) {
                    sampleTrace.add(toShieldVehicles(state));
                }
                sampleTraces.add(sampleTrace);
            }

            List<Vehicle> vehicles = sampleTraces.get(0).get(0);
            HighwayEngine shieldEngine = new HighwayEngine(dt, true, true, vehicles);
            shieldEngine.numLanes = inferNumLanes(vehicles);

            TraceBasedStarkShieldApp shield = new TraceBasedStarkShieldApp(
                    shieldEngine,
                    shieldEngine.vehicles,
                    sampleTraces,
                    true);

            boolean safe = shield.verifySafe();
            if (printDiagnostics || !safe) {
                System.out.printf("STARK training shield: action=%s samples=%d safe=%s%n",
                        action, sampleTraces.size(), safe);
                System.out.println(shield.getUnsafeDiagnosis());
            }
            return safe;
        } catch (Exception exception) {
            System.err.println("STARK training shield failed: " + exception.getMessage());
            exception.printStackTrace();
            return false;
        }
    }

    public boolean verifySafetyWithTrace(String action, String vehiclesAfterAiJson,
                                         String vehiclesAfterSlow1Json, String vehiclesAfterSlow2Json) {
        String traceJson = "[" + vehiclesAfterAiJson + "," + vehiclesAfterSlow1Json + "," + vehiclesAfterSlow2Json + "]";
        return verifySafetyWithTrace(action, traceJson);
    }

    public void setPredictFutureSeconds(int predictFutureSeconds) {
        this.predictFutureSeconds = Math.max(1, predictFutureSeconds);
    }

    public void setShieldEgoRangeMeters(double shieldEgoRangeMeters) {
        this.shieldEgoRangeMeters = Math.max(0.0, shieldEgoRangeMeters);
    }

    public void setRandomizeHiddenTargetAndCooldown(boolean randomizeHiddenTargetAndCooldown) {
        this.randomizeHiddenTargetAndCooldown = randomizeHiddenTargetAndCooldown;
    }

    public void setCheckChangeLaneToRearVehicleThreat(boolean checkChangeLaneToRearVehicleThreat) {
        this.checkChangeLaneToRearVehicleThreat = checkChangeLaneToRearVehicleThreat;
    }

    public void setReadShieldIdmCooldownTimer(boolean readShieldIdmCooldownTimer) {
        this.readShieldIdmCooldownTimer = readShieldIdmCooldownTimer;
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

    public void setPrintDiagnostics(boolean printDiagnostics) {
        this.printDiagnostics = printDiagnostics;
    }

    private List<Vehicle> toShieldVehicles(HighwayEnvVehicleState[] states) {
        List<Vehicle> vehicles = new ArrayList<>();
        for (int i = 0; i < states.length; i++) {
            HighwayEnvVehicleState state = states[i];
            Vehicle vehicle = isEgo(state)
                    ? new ControlledVehicle()
                    : new Vehicle();

            vehicle.id = String.valueOf(i);
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
            throw new IllegalArgumentException("vehiclesJson must include one vehicle with role='EGO'");
        }
        return vehicles;
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

    private HighwayAiClient.AiDecision toAiDecision(String action) {
        int actionId = parseActionId(action);
        HighwayAiClient.AiDecision decision = new HighwayAiClient.AiDecision();
        decision.action = actionId;
        decision.action_name = switch (actionId) {
            case 0 -> "LANE_LEFT";
            case 2 -> "LANE_RIGHT";
            case 3 -> "FASTER";
            case 4 -> "SLOWER";
            default -> "IDLE";
        };
        return decision;
    }

    private int parseActionId(String action) {
        if (action == null) {
            return 1;
        }
        String normalized = action.trim().toUpperCase();
        return switch (normalized) {
            case "0", "LANE_LEFT" -> 0;
            case "2", "LANE_RIGHT" -> 2;
            case "3", "FASTER" -> 3;
            case "4", "SLOWER" -> 4;
            default -> 1;
        };
    }

    public static void main(String[] args) {
        StarkForTrainingAI app = new StarkForTrainingAI();
        GatewayServer server = new GatewayServer(app);
        server.start();
        System.out.println("STARK training shield GatewayServer started.");
    }

    private static class HighwayEnvVehicleState {
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
