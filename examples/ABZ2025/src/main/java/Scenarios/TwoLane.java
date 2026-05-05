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

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.controller.ExecController;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;

import java.util.*;
import java.util.stream.Stream;

import org.apache.commons.math3.random.RandomGenerator;

public class TwoLane {
    // define constants
    private static final int EVOLUTION_SEQUENCE_SIZE = 100;


    private static final double RESPONSE_TIME = 1;
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;
    private static final double MAX_BRAKE = -5;
    private static final double MIN_BRAKE = -3;
    private static final double MAX_ACCELERATION = 5;

    // the upper lane is lane 0, the lower lane is lane 1
    private static final int FASTER = 3;
    private static final int SLOWER = 4;
    private static final int IDLE  = 1;
    private static final int LANE_LEFT = 0;
    private static final int LANE_RIGHT = 2;
    private static final int LEFT_LANE_NO = 0;
    private static final int RIGHT_LANE_NO = 1;
    private static final int SIMULATION_FREQUENCY = 15;


    //constant for IDM model
    private static final double DELTA = 4.0;
    private static final double COMFORT_ACC_MAX = 3.0;
    private static final double COMFORT_ACC_MIN = -5.0;
    private static final double TIME_WANTED = 1.5;
    private static final double DISTANCE_WANTED = 10;
    // target velocity is modeled as R.V in STARK



    // constant for MOBIL model
    private static final double LANE_CHANGE_MIN_ACC_GAIN = 0.2;
    // politeness is modeled as R.V in STARK
    private static final double LANE_CHANGE_MAX_BRAKING_IMPOSED = 2.0;
    private static final double LANE_CHANGE_DELAY = 1.0;


    // we are making an assumption that the sensor can detect 9 cars
    // that is, there are 9 cars in front of the agent
    private static final int NUMBER_OF_VEHICLES = 10;
    private static final int NUM_OF_LANES = 2;
    // vars governed by STARK
    private static final int intention = 0;

    // the first index is for the agent car
    private static final int[] speed = new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    private static final int[] accel = new int[]{11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    private static final int[] x_pos = new int[]{21, 22, 23, 24, 25, 26, 27, 28, 29, 30};
    //y_pos is an index (0/1)
    private static final int[] y_pos = new int[]{31, 32, 33, 34, 35, 36, 37, 38, 39, 40};

    // IDM updates acceleration based on target velocity and politeness
    // the first elements are useless ( agent is not controlled by IDM or MOBIL model)
    private static final int[] target_velocity = new int[]{41, 42, 43, 44, 45, 46, 47, 48, 49, 50};
    private static final int[] politeness = new int[]{51, 52, 53, 54, 55, 56, 57, 58, 59, 60};

    // these variable indicate if the corresponding car exists
    private static final int[] car_present = new int[]{61, 62, 63, 64, 65, 66, 67, 68, 69, 70};

    private static final int[] lane_change_delay_timer = new int[]{71, 72, 73, 74, 75, 76, 77, 78, 79, 80};

    // 1 if the car is changing
    private static final int [] lane_changing_indicator = new int[]{81, 82, 83, 84, 85, 86, 87, 88, 89, 90};
    // only meaningful if lane_changing_indicator is 1, indicating which lane the car is changing to
    private static final int [] desired_lane = new int[]{91, 92, 93, 94, 95, 96, 97, 98, 99, 100};
    // a car needs 0.6 - 0.8 seconds to complete a lane change
    private static final int [] changing_lane_timer = new int[]{101, 102, 103, 104, 105, 106, 107, 108, 109, 110};
    private static final int [] current_lane = new int[]{111, 112, 113, 114, 115, 116, 117, 118, 119, 120};
    private static int global_timer = 121;
    private static int [] change_lane_decision_timer = new int []{122, 123, 124, 125, 126, 127, 128, 129, 130, 131};
    private static int simulation_clock = 132;
    private static final int initialized_flag = 133;
    private static final int ego_tgspd =134;
    private static final int NUMBER_OF_VARIABLES = 135;

    private boolean isSafe = true;

     private Controller getController() {
        ControllerRegistry registry = new ControllerRegistry();

         Controller goIntermediate = Controller.doAction(
                 (_rg, _ds) -> List.of(new DataStateUpdate(intention, IDLE)),
                 registry.reference("Control")
         );
         Controller conservativeController = Controller.doAction(
                 (_rg, _ds) -> List.of(new DataStateUpdate(intention, SLOWER), new DataStateUpdate(ego_tgspd, _ds.get(ego_tgspd) - 5),
                         new DataStateUpdate(desired_lane[0], _ds.get(current_lane[0]))),
                 registry.reference("Control")
         );
         Controller initialController = Controller.doAction(
                 (_rg, _ds) -> List.of(new DataStateUpdate(initialized_flag, 1.0)),
                 registry.reference("Control")
         );
         registry.set("Control",


                         Controller.ifThenElse(
                                 (rg, ds) -> Utils.doubleEqual(ds.get(simulation_clock), SIMULATION_FREQUENCY),
                                 conservativeController,
                                 goIntermediate

                 )
         );

        return new ExecController(registry.reference("Control"));
    }
    public TwoLane(VehicleState[] vehicles, String action) {
        DataState state = getInitialState(vehicles, action);
        ControlledSystem system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);
        EvolutionSequence sequence = new EvolutionSequence(new SilentMonitor("Vehicle"), new DefaultRandomGenerator(), rg -> system, EVOLUTION_SEQUENCE_SIZE);
        printSummary(sequence, 1 );
    }
    public static void printSummary(EvolutionSequence sequence, int steps){

        SampleSet<SystemState> dss = sequence.get(steps * SIMULATION_FREQUENCY -1 );
        Stream<SystemState> stream = dss.stream();
        stream.limit(5)
                .map(ss -> {
                    DataState ds = ss.getDataState();
                    Double spd0 = ds.get(speed[0]);
                    Double xPos0 = ds.get(x_pos[0]);
                    Double lane0 = ds.get(current_lane[0]);
                    Double xPos2 = ds.get(x_pos[2]);
                    Double xPos1 = ds.get(x_pos[1]);
                    double accel2 = ds.get(accel[2]);
                    double accel1 = ds.get(accel[1]);
                    double accel0 = ds.get(accel[0]);
                    double selfTgd = ds.get(ego_tgspd);
//                    return String.format("Distance ahead: %.2f, Speed1: %.2f, Accel1: %.2f,Accel2: %.2f, xPos1: %.2f, xPos2: %.2f",
//                            dist, spd1, accel1, accel2, xPos1, xPos2);
                    //将信息全部打印
                    return String.format("Ego: Speed: %.2f, Accel0: %.2f, xPos: %.2f, Lane: %.0f, Tgd: %.2f | Car1: Speed: %.2f, Accel1: %.2f, xPos: %.2f | Car2: Speed: %.2f, Accel2: %.2f, xPos: %.2f",
                            spd0, accel0, xPos0, lane0, selfTgd,
                            ds.get(speed[1]), accel1, xPos1,
                            ds.get(speed[2]), accel2, xPos2
                    );


                })
                .forEach(System.out::println);

     }
    public boolean verify(){
        // check if the system is safe in the first 2 seconds
        return isSafe;
    }


    private int actionToInt(String action){
        System.out.println(action);
//        switch (action.toLowerCase()){
//            case "2":
//                return FASTER;
//            case "0":
//                return SLOWER;
//            case "1":
//                return IDLE;
//            default:
//                throw new IllegalArgumentException("Unknown action: " + action);
//        }
        switch(action.toLowerCase()){
            case "0":
                return LANE_LEFT;
            case "1":
                return IDLE;
            case "2":
                return LANE_RIGHT;
            case "3":
                return FASTER;
            case "4":
                return SLOWER;
            default:
                throw new IllegalArgumentException("Unknown action: " + action);
        }
    }
    private DataState getInitialState(VehicleState[] vehicles, String action) {

        Map<Integer, Double> values = new HashMap<>();
        values.put(intention, (double)actionToInt(action));

        int iMax = Math.min(vehicles.length, NUMBER_OF_VEHICLES);
        System.out.println("imax = " + iMax);
        for (int i = 0; i < iMax; i++){

            values.put(speed[i], vehicles[i].speed);
            values.put(x_pos[i], vehicles[i].dist);
            values.put(target_velocity[i], vehicles[i].targetSpeed);
            values.put(car_present[i], 1.0);
            values.put(current_lane[i], (double) vehicles[i].lane);
            values.put(accel[i], vehicles[i].acceleration);
            values.put(y_pos[i], (double) vehicles[i].lane);
            values.put(politeness[i], 0.5); // assume all cars have the same politeness
            values.put(change_lane_decision_timer[i], 0.0);
            values.put(lane_changing_indicator[i], 0.0);
            values.put(desired_lane[i], (double) vehicles[i].lane);
            values.put(changing_lane_timer[i], 0.0);
            values.put(current_lane[i], (double) vehicles[i].lane);
            values.put(lane_change_delay_timer[i], 0.0);
        }
        values.put(ego_tgspd, vehicles[0].targetSpeed);
        for (int i = iMax; i < NUMBER_OF_VEHICLES; i++){
            values.put(car_present[i], 0.0);
        }
        switch(actionToInt(action)){
            case FASTER:
                values.put(ego_tgspd, vehicles[0].targetSpeed + 5);
                break;
            case SLOWER:
                values.put(ego_tgspd, Math.max(0, vehicles[0].targetSpeed - 5));
                break;
            case IDLE:
                values.put(ego_tgspd, vehicles[0].targetSpeed);
                break;
            case LANE_LEFT:
                if (Utils.doubleEqual(intention, LANE_RIGHT)) {
                    //values.put(new DataStateUpdate(desired_lane[0], LANE_RIGHT));
                    values.put(desired_lane[0], (double) LANE_RIGHT);
                }
                break;

            case LANE_RIGHT :
                if (Utils.doubleEqual(intention, LANE_LEFT)) {
                        //values.put(new DataStateUpdate(desired_lane[0], LANE_LEFT));
                        values.put(desired_lane[0], (double)LANE_LEFT);
                }
                break;

             default:
                 throw new IllegalArgumentException("Unknown action: " + action);

        }


        values.put(initialized_flag, 1.0);
        values.put(simulation_clock, (double) SIMULATION_FREQUENCY);
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }

    public static int[] getPossibleLanes(Integer carIndex, DataState state){

        switch ((int) state.get(current_lane[carIndex])){
            case LEFT_LANE_NO -> {
                return new int[]{LANE_RIGHT};
            }
            case RIGHT_LANE_NO -> {
                return new int[]{LANE_LEFT};
            }
            default -> {
                //warning
                System.out.println("Unknown lane index: " + state.get(y_pos[carIndex]));
                return new int[]{LANE_LEFT, LANE_RIGHT};
            }
        }

    }
    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {

        List<DataStateUpdate> updates = new LinkedList<>();
        double simulationClock = state.get(simulation_clock);
        simulationClock -= 1.0;
        if(Utils.doubleEqual(simulationClock, 0.0)){
            simulationClock = SIMULATION_FREQUENCY;
        }
        updates.add(new DataStateUpdate(simulation_clock, simulationClock));

        //within a simulation step
        double time_elapsed = 1.0 / SIMULATION_FREQUENCY;
        // the NPC cars, which follow IDM and MOBIL models
        boolean switching_lane = false;
        for(int i = 0; i < NUMBER_OF_VEHICLES; i++){
            switching_lane = false;
            if(Utils.isCarPresent(i, state, car_present)){
                //compute the intended lane of the cars
                // update desire_lane and change_lane_decision_timer
                Utils.changeLanePolicy(i, state, updates, change_lane_decision_timer,
                 lane_changing_indicator,
                 current_lane,
                 desired_lane,
                 politeness,
                 (ind)->ind==0,
                 (datastate, upd)-> {
                     //deprecated, this is done by getInitialState
                 },
                 TwoLane:: getPossibleLanes,
                 speed,
                 target_velocity,
                 x_pos,
                 NUMBER_OF_VEHICLES);


                if(state.get(lane_changing_indicator[i]) > 0.5){
                    // the car is already changing lane
                    double new_changing_lane_timer = Math.max(0, state.get(changing_lane_timer[i]) - time_elapsed);
                    if (new_changing_lane_timer < 1e-6){
                        // the car finishes lane changing
                        updates.add(new DataStateUpdate(lane_changing_indicator[i], 0.0));
                        updates.add(new DataStateUpdate(current_lane[i], state.get(desired_lane[i])));
                        updates.add(new DataStateUpdate(y_pos[i], state.get(desired_lane[i])));
                        updates.add(new DataStateUpdate(change_lane_decision_timer[i], LANE_CHANGE_DELAY));
                    }
                    updates.add(new DataStateUpdate(changing_lane_timer[i], new_changing_lane_timer));
                }
                else if(state.get(desired_lane[i]) != state.get(current_lane[i])){
                    //if the car is not changing lane, and the desired lane is different from current lane
                    // start a new change lane process if the delay timer is up
                    if (state.get(change_lane_decision_timer[i]) < 1e-6){
                        updates.add(new DataStateUpdate(lane_changing_indicator[i], 1.0 - time_elapsed));
                        //!Suppse the car needs 0.7s to complete the lane change
                        updates.add(new DataStateUpdate(changing_lane_timer[i], 0.7 - time_elapsed));
                    }
                    else{
                        // decrease the change lane decision timer
                        double new_lane_change_decision_timer = Math.max(0, state.get(change_lane_decision_timer[i]) - time_elapsed);
                        updates.add(new DataStateUpdate(change_lane_decision_timer[i], new_lane_change_decision_timer));
                    }
                }
                else {
                    // the car is not changing lane, and the desired lane is the same as current lane
                    // decrease the change lane decision timer
                    double new_lane_change_decision_timer = Math.max(0, state.get(change_lane_decision_timer[i]) - time_elapsed);
                    updates.add(new DataStateUpdate(change_lane_decision_timer[i], new_lane_change_decision_timer));
                }


                if(i==0){
                    switch((int)state.get(intention)){
                        case FASTER -> {
                            // apply a positive acceleration
                            //updates.add(new DataStateUpdate(accel[0], COMFORT_ACC_MAX));
                            //updates.add(new DataStateUpdate(speed[0], state.get(speed[0]) + COMFORT_ACC_MAX * time_elapsed));
                            updates.add(new DataStateUpdate(ego_tgspd, state.get(ego_tgspd) + 5));
                            updates.add(new DataStateUpdate(accel[0],  Math.max(MAX_BRAKE, Math.min(MAX_ACCELERATION, Utils.KP_A * (state.get(ego_tgspd) - state.get(speed[0]))))));
                            updates.add(new DataStateUpdate(speed[0], state.get(speed[0]) + COMFORT_ACC_MAX * time_elapsed));
                            updates.add(new DataStateUpdate(x_pos[0], state.get(x_pos[0]) + state.get(speed[0]) * time_elapsed + 0.5 * COMFORT_ACC_MAX * time_elapsed * time_elapsed));
                        }
                        case SLOWER -> {
                            // apply a negative acceleration
                            updates.add(new DataStateUpdate(ego_tgspd, state.get(ego_tgspd) - 5));
                            updates.add(new DataStateUpdate(accel[0],  Math.max(MAX_BRAKE, Math.min(MAX_ACCELERATION, Utils.KP_A * (state.get(ego_tgspd) - state.get(speed[0]))))));
                            updates.add(new DataStateUpdate(speed[0], state.get(speed[0]) + COMFORT_ACC_MAX * time_elapsed));
                            updates.add(new DataStateUpdate(x_pos[0], state.get(x_pos[0]) + state.get(speed[0]) * time_elapsed + 0.5 * COMFORT_ACC_MAX * time_elapsed * time_elapsed));
                        }
                        default -> {
                            // maintain current target speed

                            updates.add(new DataStateUpdate(accel[0],  Math.max(MAX_BRAKE, Math.min(MAX_ACCELERATION, Utils.KP_A * (state.get(ego_tgspd) - state.get(speed[0]))))));
                            updates.add(new DataStateUpdate(speed[0], state.get(speed[0]) + COMFORT_ACC_MAX * time_elapsed));
                            updates.add(new DataStateUpdate(x_pos[0], state.get(x_pos[0]) + state.get(speed[0]) * time_elapsed + 0.5 * COMFORT_ACC_MAX * time_elapsed * time_elapsed));
                        }
                    }
                }
                else{
                    //apply IDM
                    //compute accel
                    //double a_1 = computeAccel(i, getCarInFront(i, (int) state.get(y_pos[i]), state), state);
                    double a_1 = Utils.computeAccel(i,
                            Utils.getCarInFront(i, (int) state.get(current_lane[i]), state,  current_lane,  x_pos, NUMBER_OF_VEHICLES),
                            state, (placeholder)-> false,
                            ()->0,
                            speed,
                            target_velocity,
                            x_pos);
                    //apply time elapse
                    double a_2 = Double.POSITIVE_INFINITY;
                    if(state.get(lane_changing_indicator[i]) > 0.5){
                        a_2 = Utils.computeAccel(i,
                                Utils.getCarInFront(i, (int) state.get(desired_lane[i]), state,  current_lane,  x_pos, NUMBER_OF_VEHICLES),
                                state, (placeholder)-> false,
                                ()->0,
                                speed,
                                target_velocity,
                                x_pos);
                    }
                    double a_final = Math.min(a_1, a_2);
                    // apply longitudinal acceleration
                    updates.add(new DataStateUpdate(accel[i], a_final));
                    updates.add(new DataStateUpdate(speed[i], state.get(speed[i]) + a_final * time_elapsed));
                    updates.add(new DataStateUpdate(x_pos[i], state.get(x_pos[i]) + state.get(speed[i]) * time_elapsed + 0.5 * a_final * time_elapsed * time_elapsed));
                }


            }
        }





        return updates;

    }

}
