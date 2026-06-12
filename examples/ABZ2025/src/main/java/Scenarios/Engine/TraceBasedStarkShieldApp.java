package Scenarios.Engine;

import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.SampleSet;
import it.unicam.quasylab.jspear.SystemState;
import it.unicam.quasylab.jspear.ds.DataState;
import nl.tue.Monitoring.PerceivedSystemState;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TraceBasedStarkShieldApp extends StarkShieldApp {
    private final List<List<Vehicle>> trace;

    public TraceBasedStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, List<List<Vehicle>> trace) {
        super(engine, vehicles, Math.max(1, trace.size()));
        if (trace == null || trace.isEmpty()) {
            throw new IllegalArgumentException("Trace must contain at least one state");
        }
        this.trace = trace;
        this.predictionStepCountOverride = trace.size();
        this.sequence = new FixedEvolutionSequence(toSampleSets(trace));
    }

    public TraceBasedStarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles, List<List<List<Vehicle>>> sampleTraces,
                                    boolean sampleMajor) {
        super(engine, vehicles, 1);
        if (sampleTraces == null || sampleTraces.isEmpty() || sampleTraces.get(0).isEmpty()) {
            throw new IllegalArgumentException("Sample traces must contain at least one sample and one state");
        }
        this.trace = sampleTraces.get(0);
        this.predictionStepCountOverride = sampleTraces.get(0).size();
        this.sequence = new FixedEvolutionSequence(toSampleSetsFromSampleTraces(sampleTraces));
    }

    private List<SampleSet<SystemState>> toSampleSets(List<List<Vehicle>> trace) {
        List<SampleSet<SystemState>> sampleSets = new ArrayList<>();
        for (List<Vehicle> vehiclesAtStep : trace) {
            DataState dataState = toDataState(vehiclesAtStep);
            sampleSets.add(new SampleSet<>(List.of(new PerceivedSystemState(dataState))));
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
        values.put(crashedIndex() + 5, -1.0);
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

    private static class FixedEvolutionSequence extends EvolutionSequence {
        FixedEvolutionSequence(List<SampleSet<SystemState>> sequence) {
            super(null, new DefaultRandomGenerator(), sequence);
        }
    }
}
