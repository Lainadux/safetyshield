package Scenarios.Engine;

import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class InstantBasedStarkShieldApp extends StarkShieldApp {
    public static final double DEFAULT_INSTANT_PREDICTION_SECONDS = 0.4;
    public static final double DEFAULT_AI_ACTION_SECONDS = 0.1;
    private static final double VEHICLE_LENGTH = 5.0;
    private static final double INSTANT_MIN_FRONT_GAP = 2.0;
    private static final double INSTANT_MAX_EGO_BRAKE = 3.0;

    private final double instantPredictionSeconds;
    private final double aiActionSeconds;

    public InstantBasedStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                                      double shieldEgoRangeMeters, boolean randomizeHiddenTargetAndCooldown,
                                      boolean checkChangeLaneToRearVehicleThreat,
                                      boolean readShieldIdmCooldownTimer) {
        this(engine, vehicles, predictFutureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, DEFAULT_INSTANT_PREDICTION_SECONDS,
                DEFAULT_AI_ACTION_SECONDS);
    }

    public InstantBasedStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                                      double shieldEgoRangeMeters, boolean randomizeHiddenTargetAndCooldown,
                                      boolean checkChangeLaneToRearVehicleThreat,
                                      boolean readShieldIdmCooldownTimer,
                                      double instantPredictionSeconds,
                                      double aiActionSeconds) {
        this(engine, vehicles, predictFutureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, StarkShieldApp.DEFAULT_FIX_PREDICTION,
                StarkShieldApp.DEFAULT_AGGRESSIVE_FINAL_STABILITY,
                StarkShieldApp.DEFAULT_FINAL_STABILITY_PENALTY_MODE,
                instantPredictionSeconds, aiActionSeconds, null);
    }

    public InstantBasedStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                                      double shieldEgoRangeMeters, boolean randomizeHiddenTargetAndCooldown,
                                      boolean checkChangeLaneToRearVehicleThreat,
                                      boolean readShieldIdmCooldownTimer,
                                      boolean fixPrediction,
                                      boolean aggressiveFinalStability,
                                      FinalStabilityPenaltyMode finalStabilityPenaltyMode,
                                      double instantPredictionSeconds,
                                      double aiActionSeconds) {
        this(engine, vehicles, predictFutureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, fixPrediction, aggressiveFinalStability,
                finalStabilityPenaltyMode, instantPredictionSeconds, aiActionSeconds, null);
    }

    public InstantBasedStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                                      double shieldEgoRangeMeters, boolean randomizeHiddenTargetAndCooldown,
                                      boolean checkChangeLaneToRearVehicleThreat,
                                      boolean readShieldIdmCooldownTimer,
                                      boolean fixPrediction,
                                      boolean aggressiveFinalStability,
                                      FinalStabilityPenaltyMode finalStabilityPenaltyMode,
                                      double instantPredictionSeconds,
                                      double aiActionSeconds,
                                      Long hiddenStateRandomSeed) {
        super(engine, vehicles, predictFutureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, fixPrediction, aggressiveFinalStability,
                finalStabilityPenaltyMode, hiddenStateRandomSeed);
        this.instantPredictionSeconds = instantPredictionSeconds;
        this.aiActionSeconds = aiActionSeconds;
        this.predictionStepCountOverride = Math.max(1, (int) Math.ceil(instantPredictionSeconds / this.dt));
    }

    @Override
    protected double frontVehicleStabilityPenalty(DataState state, int egoIndex, int frontIndex) {
        int egoOffset = vehicleOffset(egoIndex);
        int frontOffset = vehicleOffset(frontIndex);
        double egoX = state.get(egoOffset + VarTable.x.ordinal());
        double frontX = state.get(frontOffset + VarTable.x.ordinal());
        double egoVx = state.get(egoOffset + VarTable.vx.ordinal());
        double frontVx = state.get(frontOffset + VarTable.vx.ordinal());
        double frontGap = frontX - egoX - VEHICLE_LENGTH;
        double closingSpeed = Math.max(0.0, egoVx - frontVx);
        int egoLane = (int) state.get(egoOffset + VarTable.lane_index.ordinal());
        int frontLane = (int) state.get(frontOffset + VarTable.lane_index.ordinal());

        if (frontGap <= 0.0 && frontLane == egoLane) {
            return 1.0;
        }
        if (closingSpeed <= 0.0 && frontGap >= INSTANT_MIN_FRONT_GAP) {
            return 0.0;
        }

        double requiredGap = INSTANT_MIN_FRONT_GAP
                + closingSpeed * instantPredictionSeconds
                + closingSpeed * closingSpeed / (2.0 * INSTANT_MAX_EGO_BRAKE);
        if (frontGap >= requiredGap) {
            return 0.0;
        }
        return Math.min(1.0, Math.max(0.0, requiredGap - frontGap) / Math.max(requiredGap, 0.01));
    }

    @Override
    public List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        int numsVehicles = vehicles.size();
        List<Vehicle> localVehicles = new ArrayList<>();

        for (int i = 0; i < numsVehicles; i++) {
            localVehicles.add(stateToVehicle(state, i));
        }

        SandboxHighwayEngine sandboxEngine = new SandboxHighwayEngine();
        sandboxEngine.dt = this.dt;
        sandboxEngine.STEPS_PER_SECOND = this.STEPS_PER_SECOND;
        sandboxEngine.stepCount = state.getStep();
        sandboxEngine.runTime = state.getStep() * this.dt;
        sandboxEngine.vehicles = localVehicles;
        sandboxEngine.numLanes = this.engine.numLanes;
        sandboxEngine.idmTimeWanted = this.engine.idmTimeWanted;

        for (Vehicle v : localVehicles) {
            v.injectEngine(sandboxEngine);
        }

        try {
            planInstantStep(localVehicles, sandboxEngine, state.getStep() * this.dt);
            for (Vehicle v : localVehicles) {
                v.applyPhysics();
            }
            sandboxEngine.checkCollisions();
        } catch (Exception e) {
            updates.add(new DataStateUpdate(crashedIndex(), 1.0));
        }

        for (int i = 0; i < numsVehicles; i++) {
            Vehicle v = localVehicles.get(i);
            int offSet = vehicleOffset(i);
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
            updates.add(new DataStateUpdate(offSet + VarTable.idmCooldownTimer.ordinal(),
                    v instanceof IDMCooldownVehicle idmCooldownVehicle ? idmCooldownVehicle.idmCooldownTimer : 0.0));
            updates.add(new DataStateUpdate(offSet + VarTable.idmActionStepLength.ordinal(),
                    v instanceof IDMCooldownVehicle idmCooldownVehicle ? idmCooldownVehicle.idmActionStepLength : 0.0));
            updates.add(new DataStateUpdate(offSet + VarTable.reactionDelay.ordinal(),
                    v instanceof DelayedIDMVehicle delayedIDMVehicle ? delayedIDMVehicle.reactionDelay : 0.0));
        }

        if (state.getStep() == predictionStepCount() - 2) {
            updateFinalFrontVehicles(updates, localVehicles, sandboxEngine);
            this.finalVehicles = sandboxEngine.vehicles;
        }

        return updates;
    }

    private void planInstantStep(List<Vehicle> localVehicles, HighwayEngine sandboxEngine, double predictionTime) throws Exception {
        int egoIndex = -1;
        for (int i = 0; i < localVehicles.size(); i++) {
            if ("EGO".equals(localVehicles.get(i).role)) {
                egoIndex = i;
                break;
            }
        }

        for (int i = 0; i < localVehicles.size(); i++) {
            Vehicle vehicle = localVehicles.get(i);
            if (i == egoIndex) {
                planInstantEgo(localVehicles, sandboxEngine, i, predictionTime);
            } else {
                vehicle.plannedSteering = EngineUtils.computeSteering(vehicle);
            }
        }
    }

    private void planInstantEgo(List<Vehicle> localVehicles, HighwayEngine sandboxEngine,
                                int egoIndex, double predictionTime) throws Exception {
        Vehicle ego = localVehicles.get(egoIndex);
        if (predictionTime < aiActionSeconds) {
            ego.plannedAcceleration = EngineUtils.computeAccel(ego, localVehicles, ego.getTargetLaneIndex());
            ego.plannedSteering = EngineUtils.computeSteering(ego);
            return;
        }

        Vehicle idmEgo = new Vehicle(ego);
        idmEgo.role = "EGO";
        idmEgo.cooldownTimer = Math.max(1.0, idmEgo.cooldownTimer);
        localVehicles.set(egoIndex, idmEgo);
        idmEgo.injectEngine(sandboxEngine);
        int[] possibleLaneArray = sandboxEngine.computePossibleLanes(idmEgo);
        List<Integer> possibleLanes = new ArrayList<>();
        for (int lane : possibleLaneArray) {
            possibleLanes.add(lane);
        }
        idmEgo.setTargetLaneIndex(EngineUtils.computeTargetLane(idmEgo, localVehicles, possibleLanes, sandboxEngine));
        idmEgo.plannedAcceleration = Math.min(
                EngineUtils.computeAccel(idmEgo, localVehicles, idmEgo.getTargetLaneIndex()),
                EngineUtils.computeAccel(idmEgo, localVehicles, idmEgo.getLaneIndex())
        );
        idmEgo.plannedSteering = EngineUtils.computeSteering(idmEgo);
    }

    private void updateFinalFrontVehicles(List<DataStateUpdate> updates, List<Vehicle> localVehicles,
                                          HighwayEngine sandboxEngine) {
        try {
            Vehicle[] fronts = new Vehicle[3];
            for (Vehicle v : localVehicles) {
                if ("EGO".equals(v.role)) {
                    fronts[0] = EngineUtils.getFrontVehicle(v, sandboxEngine.vehicles, v.getLaneIndex());
                    fronts[1] = EngineUtils.getFrontVehicle(v, sandboxEngine.vehicles, v.getLaneIndex() - 1);
                    fronts[2] = EngineUtils.getFrontVehicle(v, sandboxEngine.vehicles, v.getLaneIndex() + 1);
                    break;
                }
            }
            for (int i = 0; i < fronts.length; i++) {
                updates.add(new DataStateUpdate(vehicles.size() * VarTable.values().length + 1 + i,
                        fronts[i] == null ? -1.0 : sandboxEngine.vehicles.indexOf(fronts[i])));
            }
        } catch (Exception ignored) {
        }
    }
}
