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

import com.google.gson.annotations.SerializedName;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

// this class is for HighwayEngine
public class Vehicle {
    boolean mobiling = false; //for debugging
    public int[] possible_lanes = new int[]{0, 1, 2}; // for debugging
    public Map<Integer, List<Double>> mobil = new HashMap<>(); // for debugging
    public String id ="default";
    public double politeness = 0.0;
    private HighwayEngine engine;
    public HighwayEngine getEngine() {
        if (this.engine == null) {
            throw new IllegalStateException("Engine not injected yet!");
        }
        return this.engine;
    }
    public double cooldownTimer = 0.0;
    public int target_lane_index;

    public int lane_index;
    public String role = "NPC";
    public double x, y;
    public double vx, vy;
    public double speed;
    public double heading;


    public double plannedAcceleration = 0.0;
    public double plannedSteering = 0.0;


    public final double LENGTH = 5.0;
    public final double WHEELBASE = 5.0;
    public final double WIDTH = 2.0;

    public void injectEngine(HighwayEngine engine) {

        if(!engine.vehicles.contains(this)) {
            throw new IllegalArgumentException("Vehicle must be added to the engine before injecting it!");
        }
        this.engine = engine;
    }

    @SerializedName("target_speed")
    public double targetSpeed = 25;
    public void planAction(List<Vehicle> allVehicles) throws Exception {

        this.target_lane_index = EngineUtils.computeTargetLane(this, allVehicles, List.of(0, 1), this.engine);
        this.plannedAcceleration = Math.min(EngineUtils.computeAccel(this, allVehicles, target_lane_index), EngineUtils.computeAccel(this, allVehicles, lane_index));
        this.plannedSteering = EngineUtils.computeSteering(this);
    }


    public void applyPhysics() {

        this.speed += this.plannedAcceleration * engine.dt;


        double yawRate = (this.speed / WHEELBASE) * Math.tan(this.plannedSteering);
        this.heading += yawRate * engine.dt;

        this.x += this.speed * Math.cos(this.heading) * engine.dt;
        this.y += this.speed * Math.sin(this.heading) * engine.dt;

        this.vx = this.speed * Math.cos(this.heading);
        this.vy = this.speed * Math.sin(this.heading);


        this.lane_index = (int) Math.round(this.y / 4.0);
        this.cooldownTimer += engine.dt;


        if (Double.isNaN(this.speed) || Double.isNaN(this.heading)) {
            throw new RuntimeException("NaN Virus Detected in Vehicle " + this.role +
                    "! a=" + this.plannedAcceleration + ", steer=" + this.plannedSteering);
        }

    }
    public void checkCollision() {
        List<Vehicle> environments = this.getEngine().vehicles;
        for (Vehicle other : environments) {

            if (other == this) continue;

            // 算自己和别人的绝对距离
            double dx = Math.abs(this.x - other.x);
            double dy = Math.abs(this.y - other.y);

            // 假设车辆有 LENGTH 和 WIDTH 属性 (宽度设为 2.0)
            boolean overlapX = dx < (this.LENGTH / 2.0 + other.LENGTH / 2.0);
            boolean overlapY = dy < (this.WIDTH / 2.0 + other.WIDTH / 2.0);

            if (overlapX && overlapY) {
                // 抛出带有明确责任方的异常
                String crashMsg = String.format(
                        "💥 致命碰撞！\n肇事车辆：[%s-%d] 在移动后一头撞上了 [%s-%d]！\n" +
                                "肇事车坐标 X:%.1f Y:%.1f | 被撞车坐标 X:%.1f Y:%.1f",
                        this.role, this.id,
                        other.role, other.id,
                        this.x, this.y, other.x, other.y
                );
                throw new RuntimeException(crashMsg);
            }
        }
    }


}

