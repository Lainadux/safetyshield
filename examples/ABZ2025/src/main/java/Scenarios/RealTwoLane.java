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

import it.unicam.quasylab.jspear.ds.DataState;

import java.util.ArrayList;

public class RealTwoLane {
    private static final double RESPONSE_TIME = 1;
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;
    private static final double MAX_BRAKE = -5;
    private static final double MIN_BRAKE = -3;
    private static final double MAX_ACCELERATION = 5;

    //action index
    private static final int FASTER = 3;
    private static final int SLOWER = 4;
    private static final int IDLE  = 1;
    private static final int LANE_LEFT = 0;
    private static final int LANE_RIGHT = 2;


    private static final int LEFT_LANE_NO = 0;
    private static final int RIGHT_LANE_NO = 1;
    private static final int SIMULATION_FREQUENCY = 15;

    public static void main(String[] args) {
        HighwayEngine engine = new HighwayEngine(1/SIMULATION_FREQUENCY);
        ArrayList<Vehicle> vehicles = new ArrayList<>();
        engine.step(vehicles);


    }

     private static DataState getInitialState() {
        return null;
    }
}
