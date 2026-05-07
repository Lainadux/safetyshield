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
import it.unicam.quasylab.jspear.ds.DataStateUpdate;

import java.util.ArrayList;
import java.util.List;
import java.util.function.*;

public class Utils {
    //constant for IDM model
    private static final double DELTA = 4.0;
    private static final double COMFORT_ACC_MAX = 3.0;
    private static final double COMFORT_ACC_MIN = -5.0;
    private static final double TIME_WANTED = 1.5;
    private static final double DISTANCE_WANTED = 10;
    //constant for physical dimensions of the vehicle
    private static final double VEHICLE_LENGTH = 5;

    // constant for MOBIL model
    private static final double LANE_CHANGE_MIN_ACC_GAIN = 0.2;
    // politeness is modeled as R.V in STARK
    private static final double LANE_CHANGE_MAX_BRAKING_IMPOSED = 2.0;
    private static final double LANE_CHANGE_DELAY = 1.0;

    // constants for p-controller
    private static final double TAU_ACC = 0.6;
    public static final double KP_A = 1 / TAU_ACC;

    public static boolean doubleEqual(double a, double b){
        return Math.abs(a - b) < 1e-6;
    }
    public static boolean isCarPresent(int carIndex, DataState state, int[] car_present){
        double presence = state.get(car_present[carIndex]);
        return presence > 0.5;
    }


    /**
     * compute the acceleration of a car using IDM model
     * @param thisCarIndex the index of the car we want to compute acceleration for
     * @param frontCarIndex the index of the car in front of this car, -1 if there is no car in front
     * @param state the current data state
     * @param isEgoCar a function that takes a car index and returns whether this car is the ego car
     * @param computeEgoAccel a function that computes the acceleration of the ego car, only used when this car is the ego car
     * @param speed the array of speed variable indices for all cars
     * @param target_velocity the array of target velocity variable indices for all cars
     * @param x_pos the array of x position variable indices for all cars
     * @return the acceleration of this car, bounded by [COMFORT_ACC_MIN, COMFORT_ACC_MAX]
     */
  public static double computeAccel(int thisCarIndex, int frontCarIndex, DataState state, Function<Integer, Boolean> isEgoCar, DoubleSupplier computeEgoAccel,
                                         int[] speed, int[] target_velocity, int[] x_pos) {
        if(isEgoCar.apply(thisCarIndex)){
            return computeEgoAccel.getAsDouble();
        }
        double v = state.get(speed[thisCarIndex]);
        double v0 = state.get(target_velocity[thisCarIndex]); //  (Target Velocity)


        double freeFlowTerm = 1 - Math.pow(v / v0, DELTA);

        // what if there is no car in front?
        if (frontCarIndex == -1) {
            return COMFORT_ACC_MAX * freeFlowTerm;
        }
        // if there is indeed a car in front
        double dv = v - state.get(speed[frontCarIndex]);
        double s = state.get(x_pos[frontCarIndex]) - state.get(x_pos[thisCarIndex]) - VEHICLE_LENGTH;

        //
        s = Math.max(s, 0.01);

        // s* = s0 + v*T + (v*dv) / (2*sqrt(a*b))
        double sStar = DISTANCE_WANTED + v * TIME_WANTED +
                (v * dv) / (2 * Math.sqrt(COMFORT_ACC_MAX * Math.abs(COMFORT_ACC_MIN)));


        double interactionTerm = Math.pow(sStar / s, 2);
        double acceleration = COMFORT_ACC_MAX * (freeFlowTerm - interactionTerm);

        //  [MIN_BRAKE, MAX_ACCEL]
        return Math.max(COMFORT_ACC_MIN, Math.min(COMFORT_ACC_MAX, acceleration));
    }
    /**
     * get the index of the car in specific lane that is in front of this car
     * @return the index of the car in front, -1 if there is no car in front
     */
 public static int getCarInFront(int carIndex, int lane_index, DataState state, int[] current_lane, int[] x_pos, int NUMBER_OF_VEHICLES){
        int carInFront = -1;
        double minXPos = Double.POSITIVE_INFINITY;
        for (int i = 0; i < NUMBER_OF_VEHICLES; i++){
            if(state.get(current_lane[i]) == lane_index){

                if(state.get(x_pos[i]) > state.get(x_pos[carIndex]) && state.get(x_pos[i]) < minXPos){
                    minXPos = state.get(x_pos[i]);
                    carInFront = i;
                }

            }
        }
        return carInFront;
    }
    /**
     * get the index of the car in specific lane that is behind of this car
     * @return the index of the car behind, -1 if there is no car behind
     */
 public static int getCarBehind(int carIndex, int lane_index, DataState state, int[] current_lane, int[] x_pos, int NUMBER_OF_VEHICLES) {

        int carBehind = -1;
        double maxXPos = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < NUMBER_OF_VEHICLES; i++) {
            if (state.get(current_lane[i]) == lane_index) {

                if (state.get(x_pos[i]) < state.get(x_pos[carIndex]) && state.get(x_pos[i]) > maxXPos) {
                    maxXPos = state.get(x_pos[i]);
                    carBehind = i;
                }

            }
        }
        return carBehind;
    }
    /**
     * compute the desired lane of vehicles
     *
     */
   public static void changeLanePolicy(int car_index, DataState ds, List<DataStateUpdate> updates, int[] change_lane_decision_timer, int[] lane_changing_indicator, int[] current_lane, int[] desired_lane, int[] politeness
    , Function<Integer, Boolean> isEgoCar, BiConsumer<DataState, List<DataStateUpdate>> computeEgoLane, BiFunction<Integer, DataState, int[]> getPossibleLanes,
                                       int[] speed, int[] target_velocity, int[] x_pos, int NUMBER_OF_VEHICLES
    ){
        //
       if (isEgoCar.apply(car_index))
         {
              computeEgoLane.accept(ds, updates);
         }

       else if(ds.get(change_lane_decision_timer[car_index])< 1e-6 && ds.get(lane_changing_indicator[car_index]) < 0.5){

            ArrayList<Integer> possible_lanes = new ArrayList<>();
            for(int possible_lane : getPossibleLanes.apply(car_index, ds)){
                //double current_a = computeAccel(car_index, getCarInFront(car_index, (int) ds.get(current_lane[car_index]), ds), ds);
                double current_a = computeAccel(car_index, getCarInFront(car_index, (int) ds.get(current_lane[car_index]), ds, current_lane, x_pos, NUMBER_OF_VEHICLES), ds, (i)->Boolean.FALSE, ()->0, speed, target_velocity, x_pos);
                //double new_a = computeAccel(car_index, getCarInFront(car_index, possible_lane, ds), ds);
                double new_a = computeAccel(car_index, getCarInFront(car_index, possible_lane, ds, current_lane, x_pos, NUMBER_OF_VEHICLES), ds, (i)->Boolean.FALSE, ()->0, speed, target_velocity, x_pos);
                // we are doing a favor for the rear car
                double benefit_a_old = 0;
                double benefit_a_new = 0;
               // int benefit_car_behind = getCarBehind(car_index, possible_lane, ds);
                    int benefit_car_behind = getCarBehind(car_index, possible_lane, ds, current_lane, x_pos, NUMBER_OF_VEHICLES);
                if(benefit_car_behind != -1){
                   // benefit_a_old = computeAccel(benefit_car_behind,car_index ,ds);
                   // benefit_a_new = computeAccel(benefit_car_behind, getCarInFront(benefit_car_behind, (int) ds.get(current_lane[car_index]), ds), ds);
                      benefit_a_old = computeAccel(benefit_car_behind, car_index, ds, (i)->Boolean.FALSE, ()->0, speed, target_velocity, x_pos);
                      benefit_a_new = computeAccel(benefit_car_behind, getCarInFront(benefit_car_behind, possible_lane, ds, current_lane, x_pos, NUMBER_OF_VEHICLES), ds, (i)->Boolean.FALSE, ()->0, speed, target_velocity, x_pos);
                }
                double benefit = benefit_a_new - benefit_a_old;
                // a KARMA if we cut in
                double karma_a_old = 0;
                double karma_a_new = 0;
                //int karma_car_behind = getCarBehind(car_index, possible_lane, ds);
                    int karma_car_behind = getCarBehind(car_index, possible_lane, ds, current_lane, x_pos, NUMBER_OF_VEHICLES);
                if(karma_car_behind !=-1){
                    //karma_a_old = computeAccel(karma_car_behind, getCarInFront(karma_car_behind, possible_lane, ds), ds);
                    //karma_a_new = computeAccel(karma_car_behind, car_index, ds);
                    karma_a_old = computeAccel(karma_car_behind, getCarInFront(karma_car_behind, possible_lane, ds, current_lane, x_pos, NUMBER_OF_VEHICLES), ds, (i)->Boolean.FALSE, ()->0, speed, target_velocity, x_pos);
                    karma_a_new = computeAccel(karma_car_behind, car_index, ds, (i)->Boolean.FALSE, ()->0, speed, target_velocity, x_pos);
                }
                double karma = karma_a_new - karma_a_old;
                double overall_benefit = (new_a - current_a) + ds.get(politeness[car_index]) * (benefit + karma);
                if(overall_benefit > LANE_CHANGE_MIN_ACC_GAIN && karma_a_new > -LANE_CHANGE_MAX_BRAKING_IMPOSED){
                    possible_lanes.add(possible_lane);
                }

            }

            if(possible_lanes.size()>0){
                int final_lane = possible_lanes.get(possible_lanes.size()-1);
                updates.add(new DataStateUpdate(desired_lane[car_index], final_lane));
                //updates.add(new DataStateUpdate(change_lane_decision_timer[car_index], LANE_CHANGE_DELAY));
            }

        }

    }
}
