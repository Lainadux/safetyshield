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

package RefractoredVersion.Engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class JavaHighwayEngineUtils {
    private static final double EGO_COMFORT_BRAKE_FOR_LANE_CHANGE = 2.0;
    private static final double DELTA = 4.0;
    private static final double TAU_ACC = 0.6;
    private static final double TAU_HEADING=0.2;
    private static final double TAU_LATERAL = 0.6;
    private static final double MIN_BRAKE = -3;
    private static final double MAX_ACCELERATION = 5;
    private static final double KP_A = 1 / TAU_ACC;
    private static final double KP_HEADING = 1 / TAU_HEADING;
    private static final double KP_LATERAL = 1 / TAU_LATERAL;
    private static final double COMFORT_ACC_MAX = 3.0;
    private static final double DISTANCE_WANTED = 10;
    public static final double DEFAULT_TIME_WANTED = 1.5;
    private static final double COMFORT_ACC_MIN = -5.0;
    private static final double LANE_CHANGE_MIN_ACC_GAIN = 0.2;
    private static final double LANE_CHANGE_MAX_BRAKING_IMPOSED = 2.0;

    private static final double EGO_REAR_BRAKING_GAP_BUFFER = 2.0;



    public static boolean isSafeConsideringRearVehicle(Vehicle vehicle, Vehicle rearVehicle, double newKarma, double laneChangingVehicleAcceleration) {
        if(rearVehicle == null){
            return true;
        }

        if(rearVehicle instanceof PControlledVehicle || rearVehicle instanceof NonNpcVehicle){
            //the common effect of P-Controller yields a weird situation
            //that is when the ego is braking, other vehicle would not change to the lane that npc is occupying
            //it is too ideal, we add some risk for it
            return canRearEgoBrakeComfortablyAvoidCollision(vehicle, rearVehicle, laneChangingVehicleAcceleration);
        }
        return newKarma > -LANE_CHANGE_MAX_BRAKING_IMPOSED;


    }
    private static double longitudinalSpeed(Vehicle vehicle) {
        return vehicle.vx != 0.0 ? vehicle.vx : vehicle.speed;
    }
    private static boolean canRearEgoBrakeComfortablyAvoidCollision(Vehicle frontVehicle, Vehicle egoRearVehicle,
                                                                    double frontVehicleAcceleration) {
        double bumperGap = frontVehicle.x - egoRearVehicle.x - egoRearVehicle.LENGTH;
        if (bumperGap <= EGO_REAR_BRAKING_GAP_BUFFER) {
            return false;
        }

        double egoSpeed = longitudinalSpeed(egoRearVehicle);
        double frontSpeed = longitudinalSpeed(frontVehicle);
        double relativeSpeed = egoSpeed - frontSpeed;
        double relativeAcceleration = -EGO_COMFORT_BRAKE_FOR_LANE_CHANGE - frontVehicleAcceleration;
        if (relativeSpeed <= 0.0 && relativeAcceleration <= 0.0) {
            return true;
        }

        double closingDistanceDuringBrake;
        if (relativeAcceleration < 0.0) {
            double timeUntilNoClosing = Math.max(0.0, -relativeSpeed / relativeAcceleration);
            closingDistanceDuringBrake = relativeSpeed * timeUntilNoClosing
                    + 0.5 * relativeAcceleration * timeUntilNoClosing * timeUntilNoClosing;
        } else {
            closingDistanceDuringBrake = Double.POSITIVE_INFINITY;
        }
        return bumperGap - EGO_REAR_BRAKING_GAP_BUFFER >= closingDistanceDuringBrake;
    }


        public static int computeTargetLane(Vehicle vehicle, List<Vehicle> environments, List<Integer>possibleLanes, JavaHighwayEngine engine) throws Exception {
        if (vehicle instanceof PControlledVehicle) {
//            ControlledVehicle ego = (ControlledVehicle) vehicle;
//            if(engine.stepCount% engine.STEPS_PER_SECOND == 0 ){
//                ego.signify();
//            }
//            //return ego.target_lane_index;
//            return ego.getTargetLaneIndex();
            return vehicle.getTargetLaneIndex();
        }
        else {
            //if the car is not ready to chang lane
            if(vehicle.cooldownTimer <1.0 || isChangingLane(vehicle)){
               //check if there is another driver that is also changing to the same lane
                for(Vehicle other: environments){
                    if(other == vehicle) continue;
                    if(isChangingLane(other) && other.getTargetLaneIndex() == vehicle.getTargetLaneIndex()){
                        double d =  vehicle.laneDistanceTo(other);
                        double d_star = desiredGap(vehicle, other);
                        if(0<d &&d < d_star){

                            return vehicle.getLaneIndex();
                        }

                    }
                }

//                return vehicle.target_lane_index;
                return vehicle.getTargetLaneIndex();
            }

            else {
                Map<Integer, List<Double>> mobilmap = new HashMap<>();
                //Map<Integer, Vehicle.MobilDebugInfo> mobilDebugMap = new HashMap<>();
                vehicle.mobiling = true;
                List<Integer>newLane = new ArrayList<>();
                for(int lane: engine.computePossibleLanes(vehicle)){

//                    if(lane != vehicle.lane_index){
                    if(lane != vehicle.getLaneIndex()){
                        //double current_a = computeAccel(vehicle, environments, vehicle.lane_index);
                        double current_a = computeRawAccel(vehicle, environments, vehicle.getLaneIndex());
                        double new_a = computeRawAccel(vehicle, environments, lane);
                        double benefit_a_old = 0;
                        double benefit_a_new = 0;
//                        Vehicle benifitCarBehind =  getRearVehicle(vehicle, environments, vehicle.lane_index);
                        Vehicle benifitCarBehind =  getRearVehicle(vehicle, environments, vehicle.getLaneIndex());
                        if(benifitCarBehind != null){
                            benefit_a_old = computeRawAccel(benifitCarBehind, vehicle);
                            //benefit_a_new = computeAccel(benifitCarBehind, getFrontVehicle(vehicle, environments, vehicle.lane_index));
                            benefit_a_new = computeRawAccel(benifitCarBehind, getFrontVehicle(vehicle, environments, vehicle.getLaneIndex()));
                        }
                        double benefit = benefit_a_new - benefit_a_old;

                        double karma_a_old = 0;
                        double karma_a_new = 0;

                        Vehicle targetFrontVehicle = getFrontVehicle(vehicle, environments, lane);
                        Vehicle karma_car_behind = getRearVehicle(vehicle, environments, lane);
                        if(karma_car_behind != null){
                            karma_a_old = computeRawAccel(karma_car_behind, getFrontVehicle(karma_car_behind, environments, lane));
                            karma_a_new = computeRawAccel(karma_car_behind, vehicle);

                            //System.out.println("karma_old: " + karma_a_old + ", karma_new: " + karma_a_new);
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


                        if(overall_benefit > LANE_CHANGE_MIN_ACC_GAIN && isSafeConsideringRearVehicle(vehicle, karma_car_behind, karma_a_new, new_a)){
                            newLane.add(lane);
                        }



                        mobilmap.put(lane, List.of(overall_benefit, new_a - current_a, benefit, karma_a_new));
//                        mobilDebugMap.put(lane, new Vehicle.MobilDebugInfo(
//                                lane,
//                                vehicleIdOrNone(targetFrontVehicle),
//                                vehicleIdOrNone(karma_car_behind),
//                                vehicleIdOrNone(benifitCarBehind),
//                                overall_benefit,
//                                new_a - current_a,
//                                karma,
//                                benefit
//                        ));
                    }

                }

                vehicle.mobil = mobilmap;
                //vehicle.mobilDebug = mobilDebugMap;
                vehicle.possible_lanes = newLane.stream().mapToInt(i -> i).toArray();


                //return !newLane.isEmpty() ? newLane.get(newLane.size()-1) : vehicle.lane_index;
                if(!newLane.isEmpty()){
                    vehicle.cooldownTimer = 0.0;
                    return newLane.get(newLane.size()-1);
                }
                else {

//                    return vehicle.lane_index;
                    return vehicle.getLaneIndex();
                }

            }
        }


    }
    public static boolean isChangingLane(Vehicle vehicle) {
        //System.out.println("vehicle lane index: " + vehicle.lane_index + ", target lane index: " + vehicle.target_lane_index);
        //return vehicle.lane_index != vehicle.target_lane_index;
        return vehicle.getLaneIndex() != vehicle.getTargetLaneIndex();
    }
    public static double computeRawAccel(Vehicle vehicle, Vehicle frontVehicle) throws Exception {
        if (vehicle instanceof PControlledVehicle) {
            return KP_A * (vehicle.targetSpeed - vehicle.speed);
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
            double sStar = DISTANCE_WANTED + v * DEFAULT_TIME_WANTED +
                    (v * dv) / (2 * Math.sqrt(COMFORT_ACC_MAX * Math.abs(COMFORT_ACC_MIN)));
            sStar = Math.max(sStar, DISTANCE_WANTED);
            double interactionTerm = Math.pow(sStar / s, 2);
            double acceleration = COMFORT_ACC_MAX * (freeFlowTerm - interactionTerm);
            return acceleration;
        }
    }
    public static double computeRawAccel(Vehicle vehicle, List<Vehicle> environments, int LaneNo) throws Exception {
        if (vehicle instanceof PControlledVehicle) {
            return KP_A * (vehicle.targetSpeed - vehicle.speed);
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
                double sStar = DISTANCE_WANTED + v * DEFAULT_TIME_WANTED +
                        (v * dv) / (2 * Math.sqrt(COMFORT_ACC_MAX * Math.abs(COMFORT_ACC_MIN)));
                sStar = Math.max(sStar, DISTANCE_WANTED);
                double interactionTerm = Math.pow(sStar / s, 2);
                double acceleration = COMFORT_ACC_MAX * (freeFlowTerm - interactionTerm);
                return acceleration;
            }

        }
    }
    public static double desiredGap(Vehicle thisVehicle, Vehicle frontVehicle) {

        if (frontVehicle == null) {
            return DISTANCE_WANTED;
        }
        double dv;
        double relative_vx = thisVehicle.vx - frontVehicle.vx;
        double relative_vy = thisVehicle.vy - frontVehicle.vy;
        double dir_x = Math.cos(thisVehicle.heading);
        double dir_y = Math.sin(thisVehicle.heading);
        dv = (relative_vx * dir_x) + (relative_vy * dir_y);
        double v = thisVehicle.speed;
        double s0 = DISTANCE_WANTED;
        double vT = v * DEFAULT_TIME_WANTED;
        double kineticTerm = (v * dv) / (2.0 * Math.sqrt(COMFORT_ACC_MAX * Math.abs(COMFORT_ACC_MIN)));
        double d_star = s0 + vT + kineticTerm;
        d_star = Math.max(s0 + vT, d_star);

        return d_star;
    }
    public static Vehicle getFrontVehicle(Vehicle thisCar, List<Vehicle> environments, int LaneNo) throws Exception {
        if (!environments.contains(thisCar)) {
            throw new Exception("thisCar is not in environments");
        }

        Vehicle frontVehicle = null;
        for (Vehicle v : environments) {
//            if (v.lane_index == LaneNo && v.x > thisCar.x) {
            if (v.getLaneIndex() == LaneNo && v.x > thisCar.x) {
                if (frontVehicle == null || v.x < frontVehicle.x) {
                    frontVehicle = v;
                }
            }
        }
        return frontVehicle;
    }
     public static Vehicle getRearVehicle(Vehicle thisCar, List<Vehicle> environments, int LaneNo) throws Exception {
        if (!environments.contains(thisCar)) {
            throw new Exception("thisCar is not in environments");
        }

        Vehicle rearVehicle = null;
        for (Vehicle v : environments) {
            //if (v.lane_index == LaneNo && v.x < thisCar.x) {
            if (v.getLaneIndex() == LaneNo && v.x < thisCar.x) {
                if (rearVehicle == null || v.x > rearVehicle.x) {
                    rearVehicle = v;
                }
            }
        }
        return rearVehicle;
    }
    public static double computeAccel(Vehicle vehicle, List<Vehicle> environments, int LaneNo) throws Exception {
        return clip(computeRawAccel(vehicle, environments, LaneNo));
    }
    public static double computeIdmAcceleration(Vehicle vehicle, List<Vehicle> environments) throws Exception {
        return Math.min(
                computeAccel(vehicle, environments, vehicle.getTargetLaneIndex()),
                computeAccel(vehicle, environments, vehicle.getLaneIndex())
        );
    }
    public static double clip(double accel){
        return Math.min(MAX_ACCELERATION, Math.max(MIN_BRAKE, accel));
    }

    public static double computeSteering(Vehicle vehicle) {
//        target_lane = self.road.network.get_lane(target_lane_index)
//        lane_coords = target_lane.local_coordinates(self.position)
//        lane_next_coords = lane_coords[0] + self.speed * self.TAU_PURSUIT
//        lane_future_heading = target_lane.heading_at(lane_next_coords)
//
//        # Lateral position control
//                lateral_speed_command = -self.KP_LATERAL * lane_coords[1]
//        # Lateral speed to heading
//        heading_command = np.arcsin(
//                np.clip(lateral_speed_command / utils.not_zero(self.speed), -1, 1)
//        )
//        heading_ref = lane_future_heading + np.clip(
//                heading_command, -np.pi / 4, np.pi / 4
//        )
//        # Heading control
//        heading_rate_command = self.KP_HEADING * utils.wrap_to_pi(
//                heading_ref - self.heading
//        )
//        # Heading rate to steering angle
//                slip_angle = np.arcsin(
//                np.clip(
//                        self.LENGTH / 2 / utils.not_zero(self.speed) * heading_rate_command,
//                        -1,
//                        1,
//                        )
//        )
//        steering_angle = np.arctan(2 * np.tan(slip_angle))
//        steering_angle = np.clip(
//                steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE
//        )
//        return float(steering_angle)

        double lane_coords_x = vehicle.x;
        double lane_coords_y = vehicle.y - vehicle.getTargetLaneIndex() * 4.0;
        double lane_next_coords = lane_coords_x + vehicle.speed * vehicle.TAU_PURSUIT;
        // for straight lane, it is indeed the case
        double  lane_future_heading =0;
        double lateral_speed_command = -vehicle.KP_LATERAL * lane_coords_y;
        double heading_command = Math.asin(Math.max(-1, Math.min(1, lateral_speed_command / not_zero(vehicle.speed))));
        double heading_ref = lane_future_heading + Math.max(-Math.PI / 4, Math.min(Math.PI / 4, heading_command));
        double heading_rate_command = KP_HEADING * wrapToPi(heading_ref - vehicle.heading);
        double slip_angle = Math.asin(Math.max(-1, Math.min(1, vehicle.LENGTH / 2 / not_zero(vehicle.speed) * heading_rate_command)));
        double steering_angle = Math.atan(2 * Math.tan(slip_angle));
        steering_angle = Math.max(-vehicle.MAX_STEERING_ANGLE, Math.min(vehicle.MAX_STEERING_ANGLE, steering_angle));
        return steering_angle;


    }


//    def not_zero(x: float, eps: float = 1e-2) -> float:
//            if abs(x) > eps:
//            return x
//    elif x >= 0:
//            return eps
//    else:
//            return -eps

    public static double not_zero(double x) {
        double eps = 1e-2;
        if (Math.abs(x) > eps) {
            return x;
        } else if (x >= 0) {
            return eps;
        } else {
            return -eps;
        }
    }
//    def wrap_to_pi(x: float) -> float:
//            return ((x + np.pi) % (2 * np.pi)) - np.pi
    public static double wrapToPi(double x) {
        return ((x + Math.PI) % (2 * Math.PI)) - Math.PI;
    }



}
