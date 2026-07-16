package Scenarios.Engine;

import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.List;

public class TwoStepControllerDrivenStarkShieldApp extends ControllerDrivenStarkShieldApp {
    private static final double FALLBACK_SPEED_DELTA = 5.0;

    private final int nextAction;

    public TwoStepControllerDrivenStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                                                 double shieldEgoRangeMeters,
                                                 boolean randomizeHiddenTargetAndCooldown,
                                                 boolean checkChangeLaneToRearVehicleThreat,
                                                 boolean readShieldIdmCooldownTimer,
                                                 boolean fixPrediction,
                                                 boolean aggressiveFinalStability,
                                                 FinalStabilityPenaltyMode finalStabilityPenaltyMode,
                                                 Long hiddenStateRandomSeed,
                                                 int nextAction) {
        super(engine, vehicles, predictFutureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, fixPrediction, aggressiveFinalStability,
                finalStabilityPenaltyMode, hiddenStateRandomSeed);
        this.nextAction = nextAction;
    }

    @Override
    protected List<DataStateUpdate> egoPlanUpdates(RandomGenerator rg, DataState state) {
        int egoIndex = egoVehicleIndex(state);
        if (egoIndex < 0 || !isIntegerSecondStep(rg, state)) {
            return List.of();
        }

        int offset = vehicleOffset(egoIndex);
        int lane = (int) state.get(offset + VarTable.lane_index.ordinal());
        int targetLane = (int) state.get(offset + VarTable.target_lane_index.ordinal());
        double targetSpeed = state.get(offset + VarTable.targetSpeed.ordinal());

        if (state.getStep() == STEPS_PER_SECOND) {
            targetLane = applyTargetLaneAction(nextAction, lane, targetLane);
            targetSpeed = applyTargetSpeedAction(nextAction, targetSpeed);
        } else if (state.getStep() > STEPS_PER_SECOND) {
            targetSpeed = Math.max(0.0, targetSpeed - FALLBACK_SPEED_DELTA);
        }

        double speed = state.get(offset + VarTable.speed.ordinal());
        double plannedAcceleration = clipAcceleration((targetSpeed - speed) / 0.6);
        return List.of(
                new DataStateUpdate(offset + VarTable.target_lane_index.ordinal(), targetLane),
                new DataStateUpdate(offset + VarTable.targetSpeed.ordinal(), targetSpeed),
                new DataStateUpdate(offset + VarTable.plannedAcceleration.ordinal(), plannedAcceleration)
        );
    }

    private int applyTargetLaneAction(int action, int lane, int currentTargetLane) {
        return switch (action) {
            case 0 -> Math.max(0, lane - 1);
            case 2 -> Math.min(engine.numLanes - 1, lane + 1);
            default -> currentTargetLane;
        };
    }

    private double applyTargetSpeedAction(int action, double currentTargetSpeed) {
        return switch (action) {
            case 3 -> currentTargetSpeed + 5.0;
            case 4 -> Math.max(0.0, currentTargetSpeed - 5.0);
            default -> currentTargetSpeed;
        };
    }
}
