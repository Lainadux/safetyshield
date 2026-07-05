package Scenarios.Engine;

import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.LinkedList;
import java.util.List;

public class OvertakeGateStarkShieldApp extends StarkShieldApp {
    public OvertakeGateStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                                      double shieldEgoRangeMeters, boolean randomizeHiddenTargetAndCooldown,
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

    public OvertakeGateStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, int predictFutureSeconds,
                                      double shieldEgoRangeMeters, boolean randomizeHiddenTargetAndCooldown,
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

        int currentStep = state.getStep();
        sandboxEngine.stepCount = currentStep;
        applyOvertakeGateEgoControl(localVehicles, currentStep);
        for (Vehicle v : localVehicles) {
            v.injectEngine(sandboxEngine);
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
            updates.add(new DataStateUpdate(offSet + VarTable.idmCooldownTimer.ordinal(), getIdmCooldownTimer(v)));
            updates.add(new DataStateUpdate(offSet + VarTable.idmActionStepLength.ordinal(), getIdmActionStepLength(v)));
            updates.add(new DataStateUpdate(offSet + VarTable.reactionDelay.ordinal(), getReactionDelay(v)));
            if (sandboxEngine.crashed) {
                updates.add(new DataStateUpdate(vehicles.size() * VarTable.values().length, 1.0));
            }
        }

        if (state.getStep() == predictionStepCount() - 2) {
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
                        updates.add(new DataStateUpdate(vehicles.size() * VarTable.values().length + 1 + i,
                                sandboxEngine.vehicles.indexOf(vs[i])));
                    } else {
                        updates.add(new DataStateUpdate(vehicles.size() * VarTable.values().length + 1 + i, -1.0));
                    }
                }
                this.finalVehicles = sandboxEngine.vehicles;
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        return updates;
    }

    private void applyOvertakeGateEgoControl(List<Vehicle> localVehicles, int currentStep) {
        if (currentStep <= 0 || currentStep % this.STEPS_PER_SECOND != 0) {
            return;
        }
        int decisionSecond = currentStep / this.STEPS_PER_SECOND;
        if (decisionSecond == 1) {
            return;
        }
        for (Vehicle v : localVehicles) {
            if ("EGO".equals(v.role)) {
                v.targetSpeed = Math.max(0.0, v.targetSpeed - 5.0);
            }
        }
    }
}
