package Scenarios.Engine;

import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.controller.ExecController;
import it.unicam.quasylab.jspear.controller.ProbabilisticInterleavingController;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * A STARK shield variant that keeps the existing DisTL safety formulae from
 * {@link StarkShieldApp}, but predicts traffic by letting controllers choose
 * planned accelerations and target lanes. The environment then only applies the
 * already planned acceleration/target-lane pair to the kinematic model.
 */
public class ControllerDrivenStarkShieldApp extends StarkShieldApp {
    private static final double LANE_WIDTH = 4.0;
    private static final double FALLBACK_SPEED_DELTA = 5.0;
    private static final double NORMAL_ACCELERATION_RANGE = 0.5;
    private static final double EMERGENCY_BRAKE_THRESHOLD = -2.0;
    private static final double EMERGENCY_BRAKE_LIMIT = -3.0;
    private static final double REAR_COMFORT_BRAKE = 2.0;
    private static final double MIN_CUT_IN_REAR_GAP = 5.0;
    private static final double CUT_IN_REAR_GAP_BUFFER = 2.0;
    private static final double FRONT_LOOKAHEAD_WHEN_ABSENT = 200.0;
    private static final double DISTANCE_ADVANTAGE_SCALE = 30.0;
    private static final double SPEED_ADVANTAGE_SCALE = 5.0;
    private static final double MIN_LANE_CHANGE_SCORE = 0.2;
    private static final double MAX_LANE_CHANGE_PROBABILITY = 0.85;
    private boolean predictionEgoCollisionPrinted = false;

    public ControllerDrivenStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds) {
        this(engine, vehicles, predictFutureSeconds,
                DEFAULT_SHIELD_EGO_RANGE_METERS,
                DEFAULT_RANDOMIZE_HIDDEN_TARGET_AND_COOLDOWN,
                false,
                DEFAULT_READ_SHIELD_IDM_COOLDOWN_TIMER,
                DEFAULT_FIX_PREDICTION,
                DEFAULT_AGGRESSIVE_FINAL_STABILITY,
                DEFAULT_FINAL_STABILITY_PENALTY_MODE,
                null);
    }

    public ControllerDrivenStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                                          double shieldEgoRangeMeters,
                                          boolean randomizeHiddenTargetAndCooldown,
                                          boolean checkChangeLaneToRearVehicleThreat,
                                          boolean readShieldIdmCooldownTimer,
                                          boolean fixPrediction,
                                          boolean aggressiveFinalStability,
                                          FinalStabilityPenaltyMode finalStabilityPenaltyMode) {
        this(engine, vehicles, predictFutureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, fixPrediction, aggressiveFinalStability,
                finalStabilityPenaltyMode, null);
    }

    public ControllerDrivenStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                                          double shieldEgoRangeMeters,
                                          boolean randomizeHiddenTargetAndCooldown,
                                          boolean checkChangeLaneToRearVehicleThreat,
                                          boolean readShieldIdmCooldownTimer,
                                          boolean fixPrediction,
                                          boolean aggressiveFinalStability,
                                          FinalStabilityPenaltyMode finalStabilityPenaltyMode,
                                          Long hiddenStateRandomSeed) {
        super(engine, vehicles, predictFutureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, fixPrediction, aggressiveFinalStability,
                finalStabilityPenaltyMode, hiddenStateRandomSeed);
    }

    @Override
    public Controller getController() {
        ControllerRegistry registry = new ControllerRegistry();
        List<Controller> componentControllers = new ArrayList<>();
        for (int i = 0; i < vehicles.size(); i++) {
            if (!"EGO".equals(vehicles.get(i).role)) {
                final int vehicleIndex = i;
                String name = "component_" + i;
                Controller self = registry.reference(name);
                Controller egoAction = Controller.doAction(this::egoPlanUpdates, self);
                Controller npcAction = Controller.doAction(
                        (rg, state) -> npcPlanUpdates(rg, state, vehicleIndex),
                        self
                );
                registry.set(name, Controller.ifThenElse(
                        this::isIntegerSecondStep,
                        egoAction,
                        npcAction
                ));
                componentControllers.add(self);
            }
        }
        if (componentControllers.isEmpty()) {
            String name = "ego_only_component";
            Controller self = registry.reference(name);
            registry.set(name, Controller.ifThenElse(
                    this::isIntegerSecondStep,
                    Controller.doAction(this::egoPlanUpdates, self),
                    Controller.doAction((_rg, _state) -> List.of(), self)
            ));
            componentControllers.add(self);
        }
        return new ExecController(equalInterleaving(componentControllers));
    }

    private Controller equalInterleaving(List<Controller> controllers) {
        if (controllers.isEmpty()) {
            return ControllerRegistry.NIL;
        }
        if (controllers.size() == 1) {
            return controllers.get(0);
        }
        Controller first = controllers.get(0);
        Controller rest = equalInterleaving(controllers.subList(1, controllers.size()));
        return new ProbabilisticInterleavingController(1.0 / controllers.size(), first, rest);
    }

    @Override
    public List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        int numsVehicles = vehicles.size();
        List<Vehicle> localVehicles = toLocalVehicles(state);
        SandboxHighwayEngine sandboxEngine = sandboxEngine(localVehicles, state.getStep());

        try {
            for (Vehicle vehicle : localVehicles) {
                vehicle.plannedSteering = EngineUtils.computeSteering(vehicle);
                preventReverseMotion(vehicle);
                vehicle.applyPhysics();
            }
            sandboxEngine.checkCollisions();
        } catch (Exception e) {
            printPredictedEgoCollisionStateIfPresent(state.getStep(), localVehicles);
            updates.add(new DataStateUpdate(crashedIndex(), 1.0));
        }

        if (state.getStep() == 0) {
            recordFirstInternalAccelerationUncertainty(localVehicles);
        }

        for (int i = 0; i < numsVehicles; i++) {
            Vehicle v = localVehicles.get(i);
            int offset = vehicleOffset(i);
            updates.add(new DataStateUpdate(offset + VarTable.politeness.ordinal(), v.politeness));
            updates.add(new DataStateUpdate(offset + VarTable.cooldownTimer.ordinal(), v.cooldownTimer));
            updates.add(new DataStateUpdate(offset + VarTable.target_lane_index.ordinal(), v.getTargetLaneIndex()));
            updates.add(new DataStateUpdate(offset + VarTable.lane_index.ordinal(), v.getLaneIndex()));
            updates.add(new DataStateUpdate(offset + VarTable.x.ordinal(), v.x));
            updates.add(new DataStateUpdate(offset + VarTable.y.ordinal(), v.y));
            updates.add(new DataStateUpdate(offset + VarTable.vx.ordinal(), v.vx));
            updates.add(new DataStateUpdate(offset + VarTable.vy.ordinal(), v.vy));
            updates.add(new DataStateUpdate(offset + VarTable.speed.ordinal(), v.speed));
            updates.add(new DataStateUpdate(offset + VarTable.heading.ordinal(), v.heading));
            updates.add(new DataStateUpdate(offset + VarTable.plannedAcceleration.ordinal(), v.plannedAcceleration));
            updates.add(new DataStateUpdate(offset + VarTable.plannedSteering.ordinal(), v.plannedSteering));
            updates.add(new DataStateUpdate(offset + VarTable.targetSpeed.ordinal(), v.targetSpeed));
            updates.add(new DataStateUpdate(offset + VarTable.idmCooldownTimer.ordinal(), getIdmCooldownTimer(v)));
            updates.add(new DataStateUpdate(offset + VarTable.idmActionStepLength.ordinal(), getIdmActionStepLength(v)));
            updates.add(new DataStateUpdate(offset + VarTable.reactionDelay.ordinal(), getReactionDelay(v)));
        }
        updates.addAll(updateHistoricalCutInIntentFlags(state, localVehicles));
        if (sandboxEngine.crashed) {
            printPredictedEgoCollisionStateIfPresent(state.getStep(), localVehicles);
            updates.add(new DataStateUpdate(crashedIndex(), 1.0));
        }

        if (state.getStep() == predictionStepCount() - 2) {
            updateFinalFrontVehicles(updates, localVehicles, sandboxEngine);
            this.finalVehicles = sandboxEngine.vehicles;
        }
        return updates;
    }

    private void printPredictedEgoCollisionStateIfPresent(int predictionStep, List<Vehicle> localVehicles) {
        if (predictionEgoCollisionPrinted) {
            return;
        }
        Vehicle ego = null;
        for (Vehicle vehicle : localVehicles) {
            if ("EGO".equals(vehicle.role)) {
                ego = vehicle;
                break;
            }
        }
        if (ego == null) {
            return;
        }
        for (Vehicle other : localVehicles) {
            if (other == ego) {
                continue;
            }
            if (overlaps(ego, other)) {
                predictionEgoCollisionPrinted = true;
                System.out.printf(
                        "Predicted ego collision at step=%d:%n  ego %s%n  other %s%n",
                        predictionStep,
                        formatCollisionVehicle(ego),
                        formatCollisionVehicle(other)
                );
                return;
            }
        }
    }

    private boolean overlaps(Vehicle first, Vehicle second) {
        boolean overlapX = Math.abs(first.x - second.x) < (first.LENGTH / 2.0 + second.LENGTH / 2.0);
        boolean overlapY = Math.abs(first.y - second.y) < (first.WIDTH / 2.0 + second.WIDTH / 2.0);
        return overlapX && overlapY;
    }

    private String formatCollisionVehicle(Vehicle vehicle) {
        return String.format(
                "id=%s role=%s lane=%d targetLane=%d x=%.2f y=%.2f speed=%.2f vx=%.2f vy=%.2f targetSpeed=%.2f plannedAccel=%.2f",
                vehicle.id,
                vehicle.role,
                vehicle.getLaneIndex(),
                vehicle.getTargetLaneIndex(),
                vehicle.x,
                vehicle.y,
                vehicle.speed,
                vehicle.vx,
                vehicle.vy,
                vehicle.targetSpeed,
                vehicle.plannedAcceleration
        );
    }

    private List<Vehicle> toLocalVehicles(DataState state) {
        List<Vehicle> localVehicles = new ArrayList<>();
        for (int i = 0; i < vehicles.size(); i++) {
            localVehicles.add(stateToVehicle(state, i));
        }
        return localVehicles;
    }

    private SandboxHighwayEngine sandboxEngine(List<Vehicle> localVehicles, int step) {
        SandboxHighwayEngine sandboxEngine = new SandboxHighwayEngine();
        sandboxEngine.dt = this.dt;
        sandboxEngine.STEPS_PER_SECOND = this.STEPS_PER_SECOND;
        sandboxEngine.stepCount = step;
        sandboxEngine.runTime = step * this.dt;
        sandboxEngine.numLanes = this.engine.numLanes;
        sandboxEngine.idmTimeWanted = this.engine.idmTimeWanted;
        sandboxEngine.continueAfterNpcCollision = true;
        sandboxEngine.egoCollisionOnly = true;
        sandboxEngine.vehicles = localVehicles;
        for (Vehicle vehicle : localVehicles) {
            vehicle.injectEngine(sandboxEngine);
        }
        return sandboxEngine;
    }

    private void preventReverseMotion(Vehicle vehicle) {
        if (vehicle.speed + vehicle.plannedAcceleration * this.dt < 0.0) {
            vehicle.plannedAcceleration = -vehicle.speed / this.dt;
        }
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

    protected boolean isIntegerSecondStep(RandomGenerator rg, DataState state) {
        return state.getStep() % STEPS_PER_SECOND == 0;
    }

    protected List<DataStateUpdate> egoPlanUpdates(RandomGenerator rg, DataState state) {
        int egoIndex = egoVehicleIndex(state);
        if (egoIndex < 0 || !isIntegerSecondStep(rg, state)) {
            return List.of();
        }

        int offset = vehicleOffset(egoIndex);
        double targetSpeed = state.get(offset + VarTable.targetSpeed.ordinal());
        if (state.getStep() > 0) {
            targetSpeed = Math.max(0.0, targetSpeed - FALLBACK_SPEED_DELTA);
        }
        double speed = state.get(offset + VarTable.speed.ordinal());
        double plannedAcceleration = clipAcceleration((targetSpeed - speed) / 0.6);
        return List.of(
                new DataStateUpdate(offset + VarTable.targetSpeed.ordinal(), targetSpeed),
                new DataStateUpdate(offset + VarTable.plannedAcceleration.ordinal(), plannedAcceleration)
        );
    }

    private List<DataStateUpdate> npcPlanUpdates(RandomGenerator rg, DataState state, int vehicleIndex) {
        List<Vehicle> localVehicles = toLocalVehicles(state);
        SandboxHighwayEngine sandboxEngine = sandboxEngine(localVehicles, state.getStep());
        Vehicle vehicle = localVehicles.get(vehicleIndex);
        int offset = vehicleOffset(vehicleIndex);

        int targetLane = vehicle.getTargetLaneIndex();
        boolean changedLaneTarget = false;
        if (vehicle.cooldownTimer > 1.0 && vehicle.getLaneIndex() == vehicle.getTargetLaneIndex()) {
            LaneChoice laneChoice = chooseNpcTargetLane(rg, vehicle, localVehicles, sandboxEngine);
            targetLane = laneChoice.targetLane;
            changedLaneTarget = laneChoice.changed;
        }

        double plannedAcceleration = chooseNpcAcceleration(rg, vehicle, localVehicles, targetLane);
        List<DataStateUpdate> updates = new ArrayList<>();
        updates.add(new DataStateUpdate(offset + VarTable.target_lane_index.ordinal(), targetLane));
        updates.add(new DataStateUpdate(offset + VarTable.plannedAcceleration.ordinal(), plannedAcceleration));
        if (changedLaneTarget) {
            updates.add(new DataStateUpdate(offset + VarTable.cooldownTimer.ordinal(), 0.0));
        }
        return updates;
    }

    private LaneChoice chooseNpcTargetLane(RandomGenerator rg, Vehicle vehicle, List<Vehicle> vehicles,
                                           HighwayEngine sandboxEngine) {
        int currentLane = vehicle.getLaneIndex();
        List<LaneCandidate> candidates = new ArrayList<>();
        for (int lane : sandboxEngine.computePossibleLanes(vehicle)) {
            if (lane == currentLane) {
                continue;
            }
            LaneCandidate candidate = laneCandidate(vehicle, vehicles, lane);
            if (candidate != null) {
                candidates.add(candidate);
            }
        }
        double totalScore = candidates.stream().mapToDouble(candidate -> candidate.score).sum();
        if (totalScore <= 0.0) {
            return new LaneChoice(currentLane, false);
        }
        double changeProbability = Math.min(MAX_LANE_CHANGE_PROBABILITY, totalScore / (1.0 + totalScore));
        if (rg.nextDouble() > changeProbability) {
            return new LaneChoice(currentLane, false);
        }
        double pick = rg.nextDouble() * totalScore;
        double cumulative = 0.0;
        for (LaneCandidate candidate : candidates) {
            cumulative += candidate.score;
            if (pick <= cumulative) {
                return new LaneChoice(candidate.lane, true);
            }
        }
        LaneCandidate last = candidates.get(candidates.size() - 1);
        return new LaneChoice(last.lane, true);
    }

    private LaneCandidate laneCandidate(Vehicle vehicle, List<Vehicle> vehicles, int candidateLane) {
        try {
            Vehicle rear = EngineUtils.getRearVehicle(vehicle, vehicles, candidateLane);
            if (!rearCanBrakeAvoidCollision(vehicle, rear)) {
                return null;
            }
            Vehicle currentFront = EngineUtils.getFrontVehicle(vehicle, vehicles, vehicle.getLaneIndex());
            Vehicle candidateFront = EngineUtils.getFrontVehicle(vehicle, vehicles, candidateLane);
            double currentGap = frontGap(vehicle, currentFront);
            double candidateGap = frontGap(vehicle, candidateFront);
            double currentFrontSpeed = frontSpeed(vehicle, currentFront);
            double candidateFrontSpeed = frontSpeed(vehicle, candidateFront);
            double distanceScore = Math.max(0.0, candidateGap - currentGap) / DISTANCE_ADVANTAGE_SCALE;
            double speedScore = Math.max(0.0, candidateFrontSpeed - currentFrontSpeed) / SPEED_ADVANTAGE_SCALE;
            double score = distanceScore + speedScore;
            return score >= MIN_LANE_CHANGE_SCORE ? new LaneCandidate(candidateLane, score) : null;
        } catch (Exception ignored) {
            return null;
        }
    }

    private boolean rearCanBrakeAvoidCollision(Vehicle cutInVehicle, Vehicle rearVehicle) {
        if (rearVehicle == null) {
            return true;
        }
        double gap = cutInVehicle.x - rearVehicle.x - rearVehicle.LENGTH;
        if (gap < MIN_CUT_IN_REAR_GAP) {
            return false;
        }
        double rearSpeed = longitudinalSpeed(rearVehicle);
        double frontSpeed = longitudinalSpeed(cutInVehicle);
        double closingSpeed = Math.max(0.0, rearSpeed - frontSpeed);
        double requiredGap = CUT_IN_REAR_GAP_BUFFER + closingSpeed * closingSpeed / (2.0 * REAR_COMFORT_BRAKE);
        return gap >= requiredGap;
    }

    private double chooseNpcAcceleration(RandomGenerator rg, Vehicle vehicle, List<Vehicle> vehicles, int targetLane) {
        double emergency = emergencyFrontAcceleration(vehicle, vehicles, vehicle.getLaneIndex(), targetLane);
        if (!Double.isNaN(emergency)) {
            return Math.max(EMERGENCY_BRAKE_LIMIT, Math.min(EMERGENCY_BRAKE_THRESHOLD, emergency));
        }
        return (rg.nextDouble() - rg.nextDouble()) * NORMAL_ACCELERATION_RANGE;
    }

    private double emergencyFrontAcceleration(Vehicle vehicle, List<Vehicle> vehicles, int currentLane, int targetLane) {
        double emergency = Double.NaN;
        try {
            emergency = emergencyCandidate(emergency, EngineUtils.getFrontVehicle(vehicle, vehicles, currentLane));
            emergency = emergencyCandidate(emergency, EngineUtils.getFrontVehicle(vehicle, vehicles, targetLane));
        } catch (Exception ignored) {
        }
        return emergency;
    }

    private double emergencyCandidate(double currentEmergency, Vehicle frontVehicle) {
        if (frontVehicle == null || frontVehicle.plannedAcceleration > EMERGENCY_BRAKE_THRESHOLD) {
            return currentEmergency;
        }
        if (Double.isNaN(currentEmergency)) {
            return frontVehicle.plannedAcceleration;
        }
        return Math.min(currentEmergency, frontVehicle.plannedAcceleration);
    }

    protected int egoVehicleIndex(DataState state) {
        for (int i = 0; i < vehicles.size(); i++) {
            int offset = vehicleOffset(i);
            if ((int) state.get(offset + VarTable.role.ordinal()) == 0) {
                return i;
            }
        }
        return -1;
    }

    private double frontGap(Vehicle vehicle, Vehicle frontVehicle) {
        if (frontVehicle == null) {
            return FRONT_LOOKAHEAD_WHEN_ABSENT;
        }
        return Math.max(0.0, frontVehicle.x - vehicle.x - vehicle.LENGTH);
    }

    private double frontSpeed(Vehicle vehicle, Vehicle frontVehicle) {
        if (frontVehicle == null) {
            return Math.max(vehicle.speed, vehicle.targetSpeed);
        }
        return longitudinalSpeed(frontVehicle);
    }

    private double longitudinalSpeed(Vehicle vehicle) {
        return vehicle.vx != 0.0 ? vehicle.vx : vehicle.speed;
    }

    protected double clipAcceleration(double acceleration) {
        return Math.max(EMERGENCY_BRAKE_LIMIT, Math.min(5.0, acceleration));
    }

    private record LaneChoice(int targetLane, boolean changed) {
    }

    private record LaneCandidate(int lane, double score) {
    }
}
