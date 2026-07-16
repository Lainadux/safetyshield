package Scenarios.Engine;

import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.LinkedList;
import java.util.List;

public class TwoStepStarkShieldApp extends StarkShieldApp {
    private final int nextAction;
    private final double maxDesiredSpeed;

    public TwoStepStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                                 double shieldEgoRangeMeters,
                                 boolean randomizeHiddenTargetAndCooldown,
                                 boolean checkChangeLaneToRearVehicleThreat,
                                 boolean readShieldIdmCooldownTimer,
                                 boolean fixPrediction,
                                 boolean aggressiveFinalStability,
                                 FinalStabilityPenaltyMode finalStabilityPenaltyMode,
                                 Long hiddenStateRandomSeed,
                                 int nextAction) {
        this(engine, vehicles, predictFutureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, fixPrediction, aggressiveFinalStability,
                finalStabilityPenaltyMode, hiddenStateRandomSeed, nextAction, 40.0);
    }

    public TwoStepStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                                 double shieldEgoRangeMeters,
                                 boolean randomizeHiddenTargetAndCooldown,
                                 boolean checkChangeLaneToRearVehicleThreat,
                                 boolean readShieldIdmCooldownTimer,
                                 boolean fixPrediction,
                                 boolean aggressiveFinalStability,
                                 FinalStabilityPenaltyMode finalStabilityPenaltyMode,
                                 Long hiddenStateRandomSeed,
                                 int nextAction,
                                 double maxDesiredSpeed) {
        super(engine, vehicles, predictFutureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, fixPrediction, aggressiveFinalStability,
                finalStabilityPenaltyMode, hiddenStateRandomSeed);
        this.nextAction = nextAction;
        this.maxDesiredSpeed = maxDesiredSpeed;
    }

    @Override
    public List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        int numsVehicles = vehicles.size();
        List<Vehicle> localVehicles = new LinkedList<>();

        for (int i = 0; i < numsVehicles; i++) {
            localVehicles.add(stateToVehicle(state, i));
        }

        SandboxHighwayEngine sandboxEngine = new SandboxHighwayEngine();
        sandboxEngine.dt = this.dt;
        sandboxEngine.STEPS_PER_SECOND = this.STEPS_PER_SECOND;
        sandboxEngine.vehicles = localVehicles;
        sandboxEngine.idmTimeWanted = this.engine.idmTimeWanted;
        sandboxEngine.stepCount = state.getStep();

        if (state.getStep() % this.STEPS_PER_SECOND == 0 && state.getStep() > 0) {
            for (Vehicle vehicle : localVehicles) {
                if ("EGO".equals(vehicle.role) && state.getStep() == this.STEPS_PER_SECOND) {
                    applyNextAction(vehicle, sandboxEngine);
                }
                vehicle.injectEngine(sandboxEngine);
            }
        } else {
            for (Vehicle vehicle : localVehicles) {
                vehicle.injectEngine(sandboxEngine);
            }
        }

        try {
            sandboxEngine.step();
        } catch (Exception e) {
            e.printStackTrace();
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
            if (sandboxEngine.crashed) {
                updates.add(new DataStateUpdate(crashedIndex(), 1.0));
            }
        }
        updates.addAll(updateHistoricalCutInIntentFlags(state, localVehicles));

        if (state.getStep() == predictionStepCount() - 2) {
            try {
                Vehicle[] fronts = new Vehicle[3];
                for (Vehicle v : localVehicles) {
                    if (v instanceof ProtectedControlledVehicle) {
                        fronts[0] = EngineUtils.getFrontVehicle(v, sandboxEngine.vehicles, v.lane_index);
                        fronts[1] = EngineUtils.getFrontVehicle(v, sandboxEngine.vehicles, v.lane_index - 1);
                        fronts[2] = EngineUtils.getFrontVehicle(v, sandboxEngine.vehicles, v.lane_index + 1);
                        break;
                    }
                }
                for (int i = 0; i < fronts.length; i++) {
                    updates.add(new DataStateUpdate(vehicles.size() * VarTable.values().length + 1 + i,
                            fronts[i] == null ? -1.0 : sandboxEngine.vehicles.indexOf(fronts[i])));
                }
            } catch (Exception ignored) {
            }
            this.finalVehicles = sandboxEngine.vehicles;
        }
        return updates;
    }

    private void applyNextAction(Vehicle ego, HighwayEngine sandboxEngine) {
        switch (nextAction) {
            case 0 -> ego.setTargetLaneIndex(Math.max(0, ego.getLaneIndex() - 1));
            case 2 -> ego.setTargetLaneIndex(Math.min(sandboxEngine.numLanes - 1, ego.getLaneIndex() + 1));
            case 3 -> ego.targetSpeed += 5.0;
            case 4 -> ego.targetSpeed = Math.max(0.0, ego.targetSpeed - 5.0);
            default -> {
            }
        }
        ego.targetSpeed = Math.min(maxDesiredSpeed, Math.max(0.0, ego.targetSpeed));
    }
}
