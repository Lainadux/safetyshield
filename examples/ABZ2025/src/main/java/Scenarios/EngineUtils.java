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
import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class EngineUtils {
    public static boolean isDoubleEqual(double a, double b) {
        double epsilon = 1e-4;
        return Math.abs(a - b) < epsilon;
    }
    public static final double RIGHT_LANE_BIAS = 0.3;
    public HighwayEngine createEngine(double dt) {
        return new HighwayEngine(dt);
    }
    private static final double TAU_ACC = 0.6;
    private static final double TAU_HEADING=0.2;
    private static final double TAU_LATERAL = 0.6;

    private static final double KP_A = 1 / TAU_ACC;
    private static final double KP_HEADING = 1 / TAU_HEADING;
    private static final double KP_LATERAL = 1 / TAU_LATERAL;

    private static final double LANE_CHANGE_MIN_ACC_GAIN = 0.2;
    // politeness is modeled as R.V in STARK
    private static final double LANE_CHANGE_MAX_BRAKING_IMPOSED = 2.0;

    private static final int LANE_WIDTH = 4;
    private static final double COMFORT_ACC_MAX = 3.0;
    private static final double DISTANCE_WANTED = 10;
    private static final double TIME_WANTED = 1.5;
    private static final double COMFORT_ACC_MIN = -5.0;
    private static final double DELTA = 4.0;
    private static final double MAX_BRAKE = -5;
    private static final double MIN_BRAKE = -3;
    private static final double MAX_ACCELERATION = 5;


    /**
     * return the front vehicle in the given lane, if there is no front vehicle, return null
     *
     * @param thisCar
     * @param environments
     * @param LaneNo
     * @return
     * @throws Exception
     */
    public static Vehicle getFrontVehicle(Vehicle thisCar, List<Vehicle> environments, int LaneNo) throws Exception {
        if (!environments.contains(thisCar)) {
            throw new Exception("thisCar is not in environments");
        }

        Vehicle frontVehicle = null;
        for (Vehicle v : environments) {
            if (v.lane_index == LaneNo && v.x > thisCar.x) {
                if (frontVehicle == null || v.x < frontVehicle.x) {
                    frontVehicle = v;
                }
            }
        }
        return frontVehicle;
    }

    /**
     * return the rear vehicle in the given lane, if there is no rear vehicle, return null
     *
     * @param thisCar
     * @param environments
     * @param LaneNo
     * @return
     * @throws Exception
     */
    public static Vehicle getRearVehicle(Vehicle thisCar, List<Vehicle> environments, int LaneNo) throws Exception {
        if (!environments.contains(thisCar)) {
            throw new Exception("thisCar is not in environments");
        }

        Vehicle rearVehicle = null;
        for (Vehicle v : environments) {
            if (v.lane_index == LaneNo && v.x < thisCar.x) {
                if (rearVehicle == null || v.x > rearVehicle.x) {
                    rearVehicle = v;
                }
            }
        }
        return rearVehicle;
    }

    /**
     * return the acceleration calculated by P controller
     * @param target
     * @param current
     * @return
     */
    public static double p_controller(double target, double current) {

        return clip(KP_A * (target - current));
    }
    public static double computeAccel(Vehicle vehicle, List<Vehicle> environments, int LaneNo) throws Exception {
        if (vehicle instanceof ControlledVehicle) {
            return p_controller(vehicle.targetSpeed, vehicle.speed);
        }
        else {
            double v = vehicle.vx;
            double v0 = vehicle.targetSpeed;
            double freeFlowTerm = 1 - Math.pow(v / v0, DELTA);
            Vehicle frontVehicle = getFrontVehicle(vehicle, environments, LaneNo);
            if (frontVehicle == null) {
                return COMFORT_ACC_MAX * freeFlowTerm;
            }
            else {
                double dv = v - frontVehicle.vx;
                double s = frontVehicle.x - vehicle.x - vehicle.LENGTH;
                s = Math.max(s, 0.01);
                double sStar = DISTANCE_WANTED + v * TIME_WANTED +
                        (v * dv) / (2 * Math.sqrt(COMFORT_ACC_MAX * Math.abs(COMFORT_ACC_MIN)));
                sStar = Math.max(sStar, DISTANCE_WANTED);
                double interactionTerm = Math.pow(sStar / s, 2);
                double acceleration = COMFORT_ACC_MAX * (freeFlowTerm - interactionTerm);
                return clip(acceleration);
            }

        }
    }
    public static double computeAccel(Vehicle vehicle, Vehicle frontVehicle) throws Exception {
        if (vehicle instanceof ControlledVehicle) {
            return p_controller(vehicle.targetSpeed, vehicle.speed);
        }
        double v = vehicle.vx;
        double v0 = vehicle.targetSpeed;
        double freeFlowTerm = 1 - Math.pow(v / v0, DELTA);

        if (frontVehicle == null) {
            return COMFORT_ACC_MAX * freeFlowTerm;
        } else {
            double dv = v - frontVehicle.vx;
            double s = frontVehicle.x - vehicle.x - vehicle.LENGTH;
            s = Math.max(s, 0.01);
            double sStar = DISTANCE_WANTED + v * TIME_WANTED +
                    (v * dv) / (2 * Math.sqrt(COMFORT_ACC_MAX * Math.abs(COMFORT_ACC_MIN)));
            sStar = Math.max(sStar, DISTANCE_WANTED);
            double interactionTerm = Math.pow(sStar / s, 2);
            double acceleration = COMFORT_ACC_MAX * (freeFlowTerm - interactionTerm);
            return clip(acceleration);
        }
    }

    public static double clip(double accel){
        return Math.min(MAX_ACCELERATION, Math.max(MIN_BRAKE, accel));
    }
    public static boolean isChangingLane(Vehicle vehicle) {
        //System.out.println("vehicle lane index: " + vehicle.lane_index + ", target lane index: " + vehicle.target_lane_index);
        return vehicle.lane_index != vehicle.target_lane_index;
    }
    public static int computeTargetLane(Vehicle vehicle, List<Vehicle> environments, List<Integer>possibleLanes, HighwayEngine engine) throws Exception {
        if (vehicle instanceof ControlledVehicle) {
            ControlledVehicle ego = (ControlledVehicle) vehicle;
            if(engine.stepCount% engine.STEPS_PER_SECOND == 0 ){
                ego.fetchDesiredLaneAndTargetSpeed();
            }
            return ego.target_lane_index;
        }
        else {
            //if the car is not ready to chang lane
            if(vehicle.cooldownTimer <1.0 || isChangingLane(vehicle)){
                System.out.println(isChangingLane(vehicle));
                vehicle.mobiling = false;
                System.out.println("cooldown timer: " + vehicle.cooldownTimer);
                return vehicle.target_lane_index;
            }
            else {
                Map<Integer, List<Double>> mobilmap = new HashMap<>();
                vehicle.mobiling = true;
                List<Integer>newLane = new ArrayList<>();
                for(int lane: engine.computePossibleLanes(vehicle)){

                    if(lane != vehicle.lane_index){
                        double current_a = computeAccel(vehicle, environments, vehicle.lane_index);
                        double new_a = computeAccel(vehicle, environments, lane);

                        double benefit_a_old = 0;
                        double benefit_a_new = 0;
                        Vehicle benifitCarBehind =  getRearVehicle(vehicle, environments, vehicle.lane_index);
                        if(benifitCarBehind != null){
                            benefit_a_old = computeAccel(benifitCarBehind, vehicle);
                            benefit_a_new = computeAccel(benifitCarBehind, getFrontVehicle(vehicle, environments, vehicle.lane_index));
                        }
                        double benefit = benefit_a_new - benefit_a_old;

                        double karma_a_old = 0;
                        double karma_a_new = 0;
                        Vehicle karma_car_behind = getRearVehicle(vehicle, environments, lane);
                        if(karma_car_behind != null){
                            karma_a_old = computeAccel(karma_car_behind, getFrontVehicle(karma_car_behind, environments, lane));
                            karma_a_new = computeAccel(karma_car_behind, vehicle);
                        }
                        double karma = karma_a_new - karma_a_old;
                        //european version
//                        double bias = 0.0;
//                        if (lane > vehicle.lane_index) {
//
//                            bias = RIGHT_LANE_BIAS;
//                        } else if (lane < vehicle.lane_index) {
//
//                            bias = -RIGHT_LANE_BIAS;
//                        }
//
//
//                        double overall_benefit = (new_a - current_a) + vehicle.politness * (benefit + karma) + bias;

                        // the computation of overall_benefit differs in different sources
                        double overall_benefit = (new_a - current_a) + vehicle.politeness * (benefit + karma);

                        if(overall_benefit > LANE_CHANGE_MIN_ACC_GAIN && karma_a_new > -LANE_CHANGE_MAX_BRAKING_IMPOSED){

                            newLane.add(lane);

                        }



                        mobilmap.put(lane, List.of(overall_benefit, new_a - current_a, benefit, karma));
                    }

                }

                vehicle.mobil = mobilmap;
                vehicle.possible_lanes = newLane.stream().mapToInt(i -> i).toArray();


                //return !newLane.isEmpty() ? newLane.get(newLane.size()-1) : vehicle.lane_index;
                if(!newLane.isEmpty()){
                    vehicle.cooldownTimer = 0.0;
                    return newLane.get(newLane.size()-1);
                }
                else {

                    return vehicle.lane_index;
                }
            }
        }


    }
    public static double computeSteering(Vehicle v) {

        double targetY = v.target_lane_index * LANE_WIDTH;

        double deltaY = targetY - v.y;


        double desiredVy = KP_LATERAL * deltaY;

        desiredVy = Math.max(-2.5, Math.min(2.5, desiredVy));


        double desiredHeading = 0.0;
        if (v.speed > 1.0) {
            double ratio = desiredVy / v.speed;

            ratio = Math.max(-1.0, Math.min(1.0, ratio));
            desiredHeading = Math.asin(ratio);
        }



        double deltaHeading = desiredHeading - v.heading;

        double steering = KP_HEADING * deltaHeading;

        final double MAX_STEERING = Math.PI / 4.0;
        steering = Math.max(-MAX_STEERING, Math.min(MAX_STEERING, steering));

        return steering;
    }
}
