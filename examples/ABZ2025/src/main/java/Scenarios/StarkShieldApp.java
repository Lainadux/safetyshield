/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *                Copyright (C) 2023.
 *
 * See the NOTICE file distributed with this work for additional information
 * regarding copyright ownership.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *             http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package Scenarios;

import it.unicam.quasylab.jspear.ControlledSystem;
import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.SilentMonitor;
import it.unicam.quasylab.jspear.ds.DataState;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class StarkShieldApp {

    private static final int EVOLUTION_SEQUENCE_SIZE = 50;

    HighwayEngine engine;
    List<Vehicle> vehicles;
    public StarkShieldApp(HighwayEngine engine, List<Vehicle> vehicles) {
        this.engine = engine;
        this.vehicles = vehicles;
        DataState state = this.getInitialState(vehicles);
        //ControlledSystem system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);
        //EvolutionSequence sequence = new EvolutionSequence(new SilentMonitor("Vehicle"), new DefaultRandomGenerator(), rg -> system, EVOLUTION_SEQUENCE_SIZE);
    }
    private DataState getInitialState(List<Vehicle> vehicles) {
        Map<Integer, Double> values = new HashMap<>();
        for (int i = 0; i < vehicles.size(); i++) {
            Vehicle v = vehicles.get(i);
            int offSet = i * VarTable.values().length;
            values.put(offSet + VarTable.politeness.ordinal(), v.politeness);
            values.put(offSet + VarTable.cooldownTimer.ordinal(), v.cooldownTimer);
            values.put(offSet + VarTable.target_lane_index.ordinal(), v.starked_target_lane_index);
            values.put(offSet + VarTable.lane_index.ordinal(), v.starked_lane_index);
            values.put(offSet + VarTable.x.ordinal(), v.x);
            values.put(offSet + VarTable.y.ordinal(), v.y);
            values.put(offSet + VarTable.vx.ordinal(), v.vx);
            values.put(offSet + VarTable.vy.ordinal(), v.vy);
            values.put(offSet + VarTable.speed.ordinal(), v.speed);
            values.put(offSet + VarTable.heading.ordinal(), v.heading);
            values.put(offSet + VarTable.plannedAcceleration.ordinal(), v.plannedAcceleration);
            values.put(offSet + VarTable.plannedSteering.ordinal(), v.plannedSteering);
        }


        return new DataState(vehicles.size() * VarTable.values().length, i -> values.getOrDefault(i, Double.NaN));
    }
}
