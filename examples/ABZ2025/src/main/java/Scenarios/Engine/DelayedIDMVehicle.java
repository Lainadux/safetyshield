package Scenarios.Engine;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class DelayedIDMVehicle extends IDMCooldownVehicle {
    public double reactionDelay = 0.3;
    private transient Map<String, ArrayDeque<ObservedSnapshot>> observedHistory = new HashMap<>();

    private static class ObservedSnapshot {
        private final double time;
        private final Vehicle vehicle;

        private ObservedSnapshot(double time, Vehicle vehicle) {
            this.time = time;
            this.vehicle = vehicle;
        }
    }

    public DelayedIDMVehicle() {
    }

    public DelayedIDMVehicle(Vehicle copy) {
        super(copy);
        if (copy instanceof DelayedIDMVehicle delayedIDMVehicle) {
            this.reactionDelay = delayedIDMVehicle.reactionDelay;
        }
    }

    @Override
    public Vehicle deepCopySelf() {
        DelayedIDMVehicle copy = new DelayedIDMVehicle();
        copyBaseStateTo(copy);
        copy.idmCooldownTimer = this.idmCooldownTimer;
        copy.idmActionStepLength = this.idmActionStepLength;
        copy.reactionDelay = this.reactionDelay;
        return copy;
    }

    @Override
    protected double computeIdmAcceleration(List<Vehicle> allVehicles) throws Exception {
        recordObservedStates(allVehicles);
        if (shouldReusePlannedIdmAcceleration()) {
            return this.plannedAcceleration;
        }

        List<Vehicle> delayedEnvironment = delayedEnvironment(allVehicles);
        double acceleration = Math.min(
                EngineUtils.computeAccel(this, delayedEnvironment, this.getTargetLaneIndex()),
                EngineUtils.computeAccel(this, delayedEnvironment, this.getLaneIndex())
        );
        this.idmCooldownTimer = idmActionStepLength;
        return acceleration;
    }

    private void recordObservedStates(List<Vehicle> allVehicles) {
        if (observedHistory == null) {
            observedHistory = new HashMap<>();
        }
        double time = getEngine().runTime;
        for (Vehicle vehicle : allVehicles) {
            ArrayDeque<ObservedSnapshot> history = observedHistory.computeIfAbsent(vehicle.id, _id -> new ArrayDeque<>());
            history.addLast(new ObservedSnapshot(time, vehicle.deepCopySelf()));
            while (history.size() > 500) {
                history.removeFirst();
            }
        }
    }

    private List<Vehicle> delayedEnvironment(List<Vehicle> allVehicles) {
        double targetTime = getEngine().runTime - reactionDelay;
        List<Vehicle> delayedVehicles = new ArrayList<>();
        for (Vehicle vehicle : allVehicles) {
            delayedVehicles.add(vehicle == this ? this : historicalVehicle(vehicle, targetTime));
        }
        return delayedVehicles;
    }

    private Vehicle historicalVehicle(Vehicle currentVehicle, double targetTime) {
        ArrayDeque<ObservedSnapshot> history = observedHistory == null ? null : observedHistory.get(currentVehicle.id);
        if (history == null || history.isEmpty()) {
            return currentVehicle.deepCopySelf();
        }
        ObservedSnapshot selected = history.peekFirst();
        for (ObservedSnapshot snapshot : history) {
            if (snapshot.time <= targetTime) {
                selected = snapshot;
            } else {
                break;
            }
        }
        return selected == null ? currentVehicle.deepCopySelf() : selected.vehicle.deepCopySelf();
    }
}
