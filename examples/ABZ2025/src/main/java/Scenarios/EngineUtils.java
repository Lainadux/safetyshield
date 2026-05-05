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

import java.util.List;

public class EngineUtils {
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
        double Kp = 1/0.6;
        return clip(Kp * (target - current));
    }
    public static double computeAccel(Vehicle vehicle, List<Vehicle> environments, int LaneNo) throws Exception {
        if (vehicle.role.equals("ego")) {
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
                double interactionTerm = Math.pow(sStar / s, 2);
                double acceleration = COMFORT_ACC_MAX * (freeFlowTerm - interactionTerm);
                return clip(acceleration);
            }

        }
    }

    public static double clip(double accel){
        return Math.min(MAX_ACCELERATION, Math.max(MIN_BRAKE, accel));
    }
    public static int computeTargetLane(Vehicle vehicle, List<Vehicle> environments, List<Integer>possibleLanes) throws Exception {
        return 0;


    }
    public static double computeSteering(Vehicle v) {
        // 假设标准车道宽度为 4.0 米
        final double LANE_WIDTH = 4.0;

        // 1. 找到目标车道的中心线 Y 坐标
        double targetY = v.target_lane_index * LANE_WIDTH;

        // 2. 计算横向偏差 (Lateral Error)
        double deltaY = targetY - v.y;

        // 3. P-Controller 1: 计算期望的横向速度
        // 距离目标越远，我们希望横向平移的速度越快
        double KP_LATERAL = 1.0; // 横向比例系数
        double desiredVy = KP_LATERAL * deltaY;

        // 安全限制：横向速度不能违反物理规律，限制在 [-2.5, 2.5] m/s 之间
        desiredVy = Math.max(-2.5, Math.min(2.5, desiredVy));

        // 4. 计算期望的车头偏航角 (Desired Heading)
        // 数学公式：sin(heading) = Vy / V_total。这里做小角度近似处理
        double desiredHeading = 0.0;
        if (v.speed > 1.0) { // 极低速时防抖，防止除以 0
            desiredHeading = Math.asin(desiredVy / v.speed);
        }

        // 5. P-Controller 2: 计算方向盘转角
        // 期望的车头角度 减去 当前的实际车头角度
        double KP_HEADING = 1.5; // 转向比例系数
        double deltaHeading = desiredHeading - v.heading;

        double steering = KP_HEADING * deltaHeading;

        // 6. 物理极限限制：方向盘不能无限打，限制在最大转向角 (如 45度 = PI/4)
        final double MAX_STEERING = Math.PI / 4.0;
        steering = Math.max(-MAX_STEERING, Math.min(MAX_STEERING, steering));

        return steering;
    }
}
