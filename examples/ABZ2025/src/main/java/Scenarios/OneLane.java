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
import it.unicam.quasylab.jspear.distance.AtomicDistanceExpression;
import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import org.apache.commons.math3.random.RandomGenerator;

import javax.naming.ldap.Control;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Stream;

public class OneLane {


    // define constants
    private static final double RESPONSE_TIME = 1;
    private static final double MAX_ACCELERATION = 5;
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;
    private static final double MAX_BRAKE = -5;
    private static final double MIN_BRAKE = -3;

    // the upper lane is lane 0, the lower lane is lane 1
    private static final int FASTER = 2;
    private static final int SLOWER = 0;
    private static final int IDLE  = 1;


    private static final int SIMULATION_FREQUENCY = 15;

    //constant for IDM model
    private static final double DELTA = 4.0;
    private static final double COMFORT_ACC_MAX = 3.0;
    private static final double COMFORT_ACC_MIN = -5.0;
    private static final double TIME_WANTED = 1.5;
    private static final double DISTANCE_WANTED = 10;
    // target velocity is modeled as R.V in STARK

    private static final int NUMBER_OF_VEHICLES = 5;
    private static final int NUM_OF_LANES = 2;
    // vars governed by STARK
    private static final int intention = 0;
    private static final int[] speed = new int[]{1, 2, 3, 4, 5};
    private static final int[] accel = new int[]{6, 7, 8, 9, 10};
    private static final int[] x_pos = new int[]{11, 12, 13, 14, 15};
    private static final int[] target_velocity = new int[]{16, 17, 18, 19, 20};

    private static final int[] car_present = new int[]{21, 22, 23, 24, 25};

    private static final int simulation_clock = 26;
    // only in the first simulation clock the controller plays the action, in the rest simulation cycles
    // the controller do slower
    //flag == 0 means the controller has not played the action
    private static final int initialized_flag = 27;

    private static final int distance_ahead = 28;
    private static final int rss_safety_distance = 29;
    private static final int ego_tgspd = 30;
    private static final int NUMBER_OF_VARIABLES = 99999;
    private static final int EVOLUTION_SEQUENCE_SIZE = 100;

    public boolean isSafe = true;
    private static VehicleState[] vehicles = new VehicleState[NUMBER_OF_VEHICLES];
    public OneLane(VehicleState[] vehicles, String action){

        DataState state = getInitialState(vehicles, action);
        ControlledSystem system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);
        EvolutionSequence sequence = new EvolutionSequence(new SilentMonitor("Vehicle"), new DefaultRandomGenerator(), rg -> system, EVOLUTION_SEQUENCE_SIZE);
        printSummary(sequence, 2 * SIMULATION_FREQUENCY );
        DistanceExpression noCrashing = new AtomicDistanceExpression(ds -> {
            double distanceAhead = ds.get(distance_ahead);
            return distanceAhead - calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(speed[1]), ds.get(speed[getCarInFront(1, 0, ds)]));
        }, Double::max);

    }

    private static double calculateRSSSafetyDistance(double responseTime, double rearVehicleSpeed, double frontVehicleSpeed){
        /* Formula of safety distance presented by the Responsibility-Sensitive Safety (RSS) model
         * Shalev-Shwartz, S., Shammah, S., Shashua, A.: On a formal model of safe and scalable self-driving cars.
         * CoRR abs/1708.06374 (2017), http://arxiv.org/abs/1708.0637
         */
        double d1 = responseTime * rearVehicleSpeed;
        double d2 = 0.5 * MAX_ACCELERATION*responseTime*responseTime;
        double d3 = Math.pow((rearVehicleSpeed+responseTime*MAX_ACCELERATION),2)/(2*MIN_BRAKE);
        double d4 = - (frontVehicleSpeed*frontVehicleSpeed)/(2*MAX_BRAKE);
        double rssSafetyDistance = Math.max(d1 + d2 + d3 + d4, 0);
        // The RSS model assumes vehicles as points, but ABZ case study vehicles have dimensions.
        // We add the distances from each vehicle's center to its front/rear bumpers.
        return rssSafetyDistance + VEHICLE_LENGTH;
    }

    private  void printSummary(EvolutionSequence sequence,  int stepsToPrint){
            for(int i = 0; i<stepsToPrint;i++){
                SampleSet<SystemState> dss = sequence.get(i);
                dss.stream().limit(1).forEach(ss -> {
                    DataState ds = ss.getDataState();
                    double intentionValue = ds.get(intention);
                    String intentionStr = intentionValue == FASTER ? "FASTER" : intentionValue == SLOWER ? "SLOWER" : "IDLE";
                    System.out.printf("Intention: %s, Simulation clock: %.2f, Ego speed: %.2f, Ego tgtspd: %.2f, Distance ahead: %.2f, egoAccel: %.2f%n",
                            intentionStr, ds.get(simulation_clock), ds.get(speed[0]), ds.get(ego_tgspd),ds.get(distance_ahead), ds.get(accel[0]));
                });
            }
            SampleSet<SystemState> dss = sequence.get(2 * SIMULATION_FREQUENCY - 1);

            double avgDistance = dss.stream()
                    .mapToDouble(ss -> {
                        DataState ds = ss.getDataState();
                        return ds.get(distance_ahead);
                    })
                    .average()
                    .orElse(Double.NaN);
        Stream<SystemState> stream = dss.stream();
        stream.limit(5)
                .map(ss -> {
                    DataState ds = ss.getDataState();
                    Double dist = ds.get(distance_ahead);
                    Double spd1 = ds.get(speed[1]);
                    Double xPos2 = ds.get(x_pos[2]);
                    Double xPos1 = ds.get(x_pos[1]);
                    double accel2 = ds.get(accel[2]);
                    double accel1 = ds.get(accel[1]);
                    double accel0 = ds.get(accel[0]);
                    double selfTgd = ds.get(ego_tgspd);
//                    return String.format("Distance ahead: %.2f, Speed1: %.2f, Accel1: %.2f,Accel2: %.2f, xPos1: %.2f, xPos2: %.2f",
//                            dist, spd1, accel1, accel2, xPos1, xPos2);
                   //将信息全部打印
                    return String.format("Distance ahead: %.2f, Speed1: %.2f, Accel1: %.2f, Accel2: %.2f, xPos1: %.2f, xPos2: %.2f, Accel0: %.2f, EgoTgtSpd: %.2f",
                            dist, spd1, accel1, accel2, xPos1, xPos2, accel0, selfTgd);


                })
                .forEach(System.out::println);
//        System.out.println("Average distance between the two cars in the first 2 seconds: " + avgDistance);
            isSafe = avgDistance > 40;




    }
    public boolean verify(){
        // check if the system is safe in the first 2 seconds
        return isSafe;
    }

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double simulationClock = state.get(simulation_clock);
        simulationClock -= 1.0;
        if(doubleEqual(simulationClock, 0.0)){
            simulationClock = SIMULATION_FREQUENCY;
        }
        updates.add(new DataStateUpdate(simulation_clock, simulationClock));
        //计算每辆车的加速度
        for (int i = 0; i < NUMBER_OF_VEHICLES; i++){
            if(isCarPresent(i, state)){
                // its single lane, so lane index is always 0
                int lane_index = 0;
                int carInFront = getCarInFront(i, lane_index, state);
                //double accel = computeAccel(i, carInFront, state);
                double accel = Utils.computeAccel(i, carInFront, state, OneLane::isEgoCar, () -> Math.max(MAX_BRAKE, Math.min(MAX_ACCELERATION, Utils.KP_A * (state.get(ego_tgspd) - state.get(speed[0]))))
                , speed, target_velocity, x_pos);
                updates.add(new DataStateUpdate(OneLane.accel[i], accel));
                // update speed and position based on the acceleration
                double newSpeed = state.get(speed[i]) + accel * (1.0 / SIMULATION_FREQUENCY);
                newSpeed = Math.max(0, newSpeed); // speed cannot be negative
                updates.add(new DataStateUpdate(speed[i], newSpeed));
                double newXPos = state.get(x_pos[i]) + newSpeed * (1.0 / SIMULATION_FREQUENCY);
                updates.add(new DataStateUpdate(x_pos[i], newXPos));

            }
        }
        // 计算 distance_ahead for the ego car (car 0)
        int carInFront = getCarInFront(0, 0, state);
        double distanceAhead = carInFront == -1 ? Double.POSITIVE_INFINITY :
                state.get(x_pos[carInFront]) - state.get(x_pos[0]) - VEHICLE_LENGTH;
        updates.add(new DataStateUpdate(distance_ahead, distanceAhead));



        return updates;
    }
    /**
     * get the index of the car in specific lane that is behind of this car
     * @return the index of the car behind, -1 if there is no car behind
     */
    private static int getCarBehind(int carIndex, int lane_index, DataState state){

        int carBehind = -1;
        double maxXPos = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < NUMBER_OF_VEHICLES; i++){


                if(state.get(x_pos[i]) < state.get(x_pos[carIndex]) && state.get(x_pos[i]) > maxXPos){
                    maxXPos = state.get(x_pos[i]);
                    carBehind = i;
                }


        }


        return carBehind;
    }
    /**
     * get the index of the car in specific lane that is in front of this car
     * @return the index of the car in front, -1 if there is no car in front
     */
    private static int getCarInFront(int carIndex, int lane_index, DataState state){
        int carInFront = -1;
        double minXPos = Double.POSITIVE_INFINITY;
        for (int i = 0; i < NUMBER_OF_VEHICLES; i++){


                if(state.get(x_pos[i]) > state.get(x_pos[carIndex]) && state.get(x_pos[i]) < minXPos){
                    minXPos = state.get(x_pos[i]);
                    carInFront = i;
                }


        }
        return carInFront;
    }
    private static boolean isEgoCar(int carIndex){
        return carIndex == 0;
    }
    /**
     * compute the (clipped )acceleration of a car based on IDM model
     * return the acceleration of the car
     * if the car does not exist, return -1
     */





    private int actionToInt(String action){
        // action is 0/1/2 for slower/idle/faster
        switch (action.toLowerCase()){
            case "2":
                return FASTER;
            case "0":
                return SLOWER;
            case "1":
                return IDLE;
            default:
                throw new IllegalArgumentException("Unknown action: " + action);
        }
    }
    private static boolean doubleEqual(double a, double b){
        return Math.abs(a - b) < 1e-6;
    }
    private DataState getInitialState(VehicleState[] vehicles, String action) {
        Map<Integer, Double> values = new HashMap<>();
        values.put(intention, (double)actionToInt(action));

        int iMax = Math.min(vehicles.length, NUMBER_OF_VEHICLES);
        for (int i = 0; i < iMax; i++){
            values.put(speed[i], vehicles[i].speed);
            values.put(x_pos[i], vehicles[i].dist);
            values.put(target_velocity[i], vehicles[i].targetSpeed);
            values.put(car_present[i], 1.0);
            values.put(accel[i], vehicles[i].acceleration);
        }
        //
        values.put(ego_tgspd, vehicles[0].targetSpeed);

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
        }

        for (int i = iMax; i < NUMBER_OF_VEHICLES; i++){
            values.put(car_present[i], 0.0);
        }
        values.put(simulation_clock, (double) SIMULATION_FREQUENCY -1);
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }
    private Controller getController() {
        ControllerRegistry registry = new ControllerRegistry();

        Controller goFaster = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, FASTER)),
                registry.reference("Control"));

        Controller goSlower = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, SLOWER)),
                registry.reference("Control")
        );

        Controller goIdle = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, IDLE)),
                registry.reference("Control")
        );
        //
        Controller goIntermediate = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, IDLE)),
                registry.reference("Control")
        );
        Controller conservativeController = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, SLOWER), new DataStateUpdate(ego_tgspd, _ds.get(ego_tgspd) - 5)),
                registry.reference("Control")
        );
        Controller initialController = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(initialized_flag, 1.0)),
                registry.reference("Control")
        );
      registry.set("Control",
                Controller.ifThenElse(
                        (rg, ds) -> doubleEqual(ds.get(initialized_flag), 0),
                        initialController,
                        Controller.ifThenElse(
                                (rg, ds) -> doubleEqual(ds.get(simulation_clock), SIMULATION_FREQUENCY),
                                conservativeController,
                                goIntermediate
                        )
                )
        );







//        registry.set("Control",
//
//                Controller.ifThenElse(
//                        (rg, ds) -> !doubleEqual(ds.get(simulation_clock), SIMULATION_FREQUENCY),
//                        goIdle,
//                        Controller.ifThenElse(
//                                (rg, ds) -> doubleEqual(ds.get(decision_flag), 1.0),
//                                goIdle,
//                                Controller.ifThenElse(
//                                        (rg, ds) -> doubleEqual(ds.get(intention), FASTER),
//                                        goFaster,
//                                        Controller.ifThenElse(
//                                                (rg, ds) -> doubleEqual(ds.get(intention), SLOWER),
//                                                goSlower,
//                                                goIdle))
//
//                        )
//                )
//        );


       return new ExecController(registry.reference("Control"));
    }

    // RSS distance

    private static boolean isCarPresent(int carIndex, DataState state){
        double presence = state.get(car_present[carIndex]);
        return presence > 0.5;
    }
}
