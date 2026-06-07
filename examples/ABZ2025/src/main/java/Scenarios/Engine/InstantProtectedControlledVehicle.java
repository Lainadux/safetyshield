package Scenarios.Engine;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class InstantProtectedControlledVehicle extends ProtectedControlledVehicle {
    public static final double DEFAULT_AI_DECISION_INTERVAL_SECONDS = 0.25;
    private static final double LANE_WIDTH = 4.0;
    private static final double LANE_CENTER_TOLERANCE = 0.25;

    public double aiDecisionIntervalSeconds = DEFAULT_AI_DECISION_INTERVAL_SECONDS;
    private boolean idmFallbackActive = false;

    public InstantProtectedControlledVehicle() {
        super();
    }

    public InstantProtectedControlledVehicle(Vehicle original) {
        super(original);
        if (original instanceof InstantProtectedControlledVehicle instantProtectedControlledVehicle) {
            this.aiDecisionIntervalSeconds = instantProtectedControlledVehicle.aiDecisionIntervalSeconds;
            this.idmFallbackActive = instantProtectedControlledVehicle.idmFallbackActive;
        }
    }

    public boolean isAiDecisionStep() {
        HighwayEngine engine = this.getEngine();
        int intervalSteps = Math.max(1, (int) Math.round(aiDecisionIntervalSeconds / engine.dt));
        return engine.stepCount % intervalSteps == 0;
    }

    public boolean isLaneChangeSettled() {
        return this.getLaneIndex() == this.getTargetLaneIndex()
                && Math.abs(this.y - this.getTargetLaneIndex() * LANE_WIDTH) <= LANE_CENTER_TOLERANCE;
    }

    public void fetchInstantDesiredLaneAndTargetSpeed() {
        if (!isAiDecisionStep()) {
            return;
        }
        try {
            this.applyAiAction(HighwayAiClient.getInstance().decide(this));
        } catch (Exception e) {
            System.err.println("AI decision failed, falling back to random action: " + e.getMessage());
            this.randomActionGenerator();
        }
    }

    public void fetchInstantRandomDesiredLaneAndTargetSpeed(Random random) {
        if (isAiDecisionStep()) {
            this.applyRandomDecision(random);
        }
    }

    public void activateIdmFallback(double ignoredFallbackTargetSpeed) {
        this.idmFallbackActive = true;
        this.targetSpeed = this.speed;
        this.setTargetLaneIndex(this.getLaneIndex());
    }

    public boolean isIdmFallbackActive() {
        return idmFallbackActive;
    }

    @Override
    protected void applyAiAction(HighwayAiClient.AiDecision decision) {
        this.idmFallbackActive = false;
        super.applyAiAction(decision);
    }

    @Override
    public void planAction(List<Vehicle> allVehicles) throws Exception {
        if (!idmFallbackActive) {
            super.planAction(allVehicles);
            return;
        }

        Vehicle idmEgo = new Vehicle(this);
        idmEgo.role = "EGO";
        idmEgo.setTargetLaneIndex(idmEgo.getLaneIndex());

        List<Vehicle> fallbackVehicles = new ArrayList<>();
        for (Vehicle vehicle : allVehicles) {
            fallbackVehicles.add(vehicle == this ? idmEgo : vehicle.deepCopySelf());
        }

        SandboxHighwayEngine fallbackEngine = new SandboxHighwayEngine();
        HighwayEngine realEngine = this.getEngine();
        fallbackEngine.dt = realEngine.dt;
        fallbackEngine.STEPS_PER_SECOND = realEngine.STEPS_PER_SECOND;
        fallbackEngine.stepCount = realEngine.stepCount;
        fallbackEngine.runTime = realEngine.runTime;
        fallbackEngine.numLanes = realEngine.numLanes;
        fallbackEngine.idmTimeWanted = realEngine.idmTimeWanted;
        fallbackEngine.vehicles = fallbackVehicles;
        for (Vehicle vehicle : fallbackVehicles) {
            vehicle.injectEngine(fallbackEngine);
        }

        idmEgo.plannedAcceleration = EngineUtils.computeAccel(idmEgo, fallbackVehicles, idmEgo.getLaneIndex());
        idmEgo.plannedSteering = EngineUtils.computeSteering(idmEgo);

        this.setTargetLaneIndex(idmEgo.getTargetLaneIndex());
        this.plannedAcceleration = idmEgo.plannedAcceleration;
        this.plannedSteering = idmEgo.plannedSteering;
    }
}
