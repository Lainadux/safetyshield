package Scenarios;

import Scenarios.Engine.ControlledVehicle;
import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.ProtectedControlledVehicle;
import Scenarios.Engine.StarkShieldApp;
import Scenarios.Engine.TwoStepStarkShieldApp;
import Scenarios.Engine.Vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class SafeController {
    private static final int ACTION_LANE_RIGHT = 2;
    private static final int ACTION_FASTER = 3;
    private static final int ACTION_SLOWER = 4;

    private final List<SafeControllerActionEvaluator> evaluators = List.of(
            new EvaluateFaster(),
            new EvaluateChangeRight(),
            new EvaluateIdle(),
            new EvaluateChangeLeft()
    );

    private int cachedProposal = ACTION_FASTER;
    private Result lastResult = null;

    public Result selectCurrentActionAndCacheNext(HighwayEngine realWorld,
                                                  ProtectedControlledVehicle ego,
                                                  StarkScenarioRunner.RunOptions options,
                                                  Map<String, Double> previousDecisionSpeedByVehicleId,
                                                  long shieldDecisionIndex) {
        int proposedAction = cachedProposal;
        return evaluateCurrentActionAndCacheNext(
                proposedAction,
                realWorld,
                ego,
                options,
                previousDecisionSpeedByVehicleId,
                shieldDecisionIndex
        );
    }

    private Result evaluateCurrentActionAndCacheNext(int currentAction,
                                                     HighwayEngine realWorld,
                                                     ProtectedControlledVehicle ego,
                                                     StarkScenarioRunner.RunOptions options,
                                                     Map<String, Double> previousDecisionSpeedByVehicleId,
                                                     long shieldDecisionIndex) {
        Result firstRejectedResult = null;
        System.out.printf("---- SafeController evaluation group current=%s ----%n",
                actionName(currentAction));
        for (SafeControllerActionEvaluator evaluator : evaluators) {
            int nextAction = evaluator.action(ego, realWorld);
            StarkShieldApp shield = createTwoStepShield(realWorld, options,
                    previousDecisionSpeedByVehicleId, shieldDecisionIndex, currentAction, nextAction);
            boolean safe = shield.verifySafe();
            Result result = new Result(currentAction, nextAction, evaluator.name(), safe, shield);
            if (firstRejectedResult == null) {
                firstRejectedResult = result;
            }
            System.out.printf("SafeController evaluate current=%s nextCandidate=%s safe=%s%n",
                    actionName(currentAction), evaluator.name(), safe);
            System.out.printf("SafeController diagnosis current=%s nextCandidate=%s:%n%s%n",
                    actionName(currentAction), evaluator.name(), shield.getUnsafeDiagnosis());
            System.out.println("------------");
            if (safe) {
                cachedProposal = nextAction;
                lastResult = result;
                System.out.printf("---- SafeController evaluation group current=%s end ----%n",
                        actionName(currentAction));
                return result;
            }
        }

        cachedProposal = ACTION_LANE_RIGHT;
        lastResult = new Result(ACTION_SLOWER, ACTION_LANE_RIGHT, "LANE_RIGHT", false,
                firstRejectedResult == null ? null : firstRejectedResult.shield());
        System.out.printf("SafeController proposal=%s rejected for all next candidates; executing %s and cached next=%s%n",
                actionName(currentAction), actionName(ACTION_SLOWER), actionName(cachedProposal));
        System.out.printf("---- SafeController evaluation group current=%s end ----%n",
                actionName(currentAction));
        return lastResult;
    }

    public Result lastResult() {
        return lastResult;
    }

    private StarkShieldApp createTwoStepShield(HighwayEngine realWorld,
                                               StarkScenarioRunner.RunOptions options,
                                               Map<String, Double> previousDecisionSpeedByVehicleId,
                                               long shieldDecisionIndex,
                                               int currentAction,
                                               int nextAction) {
        Long hiddenStateSeed = options.shieldHiddenStateRandomSeed == null
                ? null
                : options.shieldHiddenStateRandomSeed + shieldDecisionIndex;
        List<Vehicle> shieldVehicles = buildShieldVehicles(realWorld.vehicles, previousDecisionSpeedByVehicleId);
        HighwayEngine shieldEngine = new HighwayEngine(realWorld.dt, true, true, shieldVehicles);
        shieldEngine.idmTimeWanted = realWorld.idmTimeWanted;
        applyActionToShieldEgo(shieldEngine.vehicles, currentAction, options.maxDesiredSpeed);
        StarkShieldApp.setAggressiveV2MaxStableRelativeSpeed(
                options.aggressiveV2MaxStableRelativeSpeed);
        StarkShieldApp.setAggressiveV3TtcThreshold(
                options.aggressiveV3TtcThreshold);
        StarkShieldApp.FinalStabilityPenaltyMode penaltyMode = options.finalStabilityMode == null
                ? StarkShieldApp.FinalStabilityPenaltyMode.AGGRESSIVE_V2
                : options.resolvedFinalStabilityPenaltyMode();
        return new TwoStepStarkShieldApp(
                shieldEngine,
                shieldEngine.vehicles,
                Math.max(2, options.shieldPredictFutureSeconds),
                options.shieldEgoRangeMeters,
                options.randomizeShieldHiddenTargetAndCooldown,
                options.checkChangeLaneToRearVehicleThreat,
                options.readShieldIdmCooldownTimer,
                options.fixPrediction,
                true,
                penaltyMode,
                hiddenStateSeed,
                nextAction,
                options.maxDesiredSpeed
        );
    }

    private void applyActionToShieldEgo(List<Vehicle> shieldVehicles, int action, double maxDesiredSpeed) {
        for (int i = 0; i < shieldVehicles.size(); i++) {
            Vehicle vehicle = shieldVehicles.get(i);
            if (vehicle instanceof ControlledVehicle controlledVehicle) {
                controlledVehicle.applyDecision(action);
                clampTargetSpeed(controlledVehicle, maxDesiredSpeed);
                return;
            }
            if ("EGO".equals(vehicle.role) || "0".equals(vehicle.id)) {
                ControlledVehicle controlledVehicle = vehicle.ascendAsControlledVehicle();
                controlledVehicle.applyDecision(action);
                clampTargetSpeed(controlledVehicle, maxDesiredSpeed);
                shieldVehicles.set(i, controlledVehicle);
                return;
            }
        }
    }

    private void clampTargetSpeed(Vehicle vehicle, double maxDesiredSpeed) {
        vehicle.targetSpeed = Math.min(maxDesiredSpeed, Math.max(0.0, vehicle.targetSpeed));
    }

    private List<Vehicle> buildShieldVehicles(List<Vehicle> realWorldVehicles,
                                              Map<String, Double> previousDecisionSpeedByVehicleId) {
        List<Vehicle> vehicles = new ArrayList<>();
        for (Vehicle v : realWorldVehicles) {
            Vehicle copy;
            if (v instanceof ControlledVehicle) {
                ControlledVehicle controlledCopy = v.deepCopySelf().ascendAsControlledVehicle();
                controlledCopy.simulated = true;
                copy = controlledCopy;
            } else {
                copy = v.deepCopySelf();
            }
            copy.previousSecondSpeed = previousDecisionSpeedByVehicleId.getOrDefault(copy.id, Double.NaN);
            vehicles.add(copy);
        }
        return vehicles;
    }

    static String actionName(int action) {
        return switch (action) {
            case 0 -> "LANE_LEFT";
            case 2 -> "LANE_RIGHT";
            case 3 -> "FASTER";
            case 4 -> "SLOWER";
            default -> "IDLE";
        };
    }

    public record Result(int currentAction, int nextAction, String selectedEvaluator, boolean selectedByShield,
                         StarkShieldApp shield) {
    }
}
