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

import java.util.List;

// this class is for HighwayEngine
public class Vehicle {
    public double cooldownTimer = 1.0;
    // 物理状态
    public int target_lane_index;
    public int lane_index;
    public String role = "NPC";
    public double x, y;
    public double vx, vy;
    public double speed;
    public double heading; // 车身偏航角

    // 内部缓存：这一帧决定要做的动作
    private double plannedAcceleration = 0.0;
    private double plannedSteering = 0.0;

    public final double LENGTH = 5.0; // 车长
    public final double WHEELBASE = 2.5; // 轴距

    //this attribute is for IDM
    @SerializedName("target_speed")
    public double targetSpeed;
    public void planAction(List<Vehicle> allVehicles) throws Exception {
        // 1. 观察周围环境（遍历 allVehicles 找到前车）
        // 2. 运行 IDM 模型计算油门
        // 3. 运行纯跟踪 / PD 控制器计算方向盘转角

        // ⚠️ 关键：只把结果存起来，绝对不在这里修改坐标！
        this.plannedAcceleration = EngineUtils.computeAccel(this, allVehicles, target_lane_index);
        this.target_lane_index = EngineUtils.computeTargetLane(this, allVehicles, List.of(0, 1));

        this.plannedSteering = EngineUtils.computeSteering(this)/* 算出的方向盘转角 */;
    }

    // === 阶段二：肉体执行 (自行车模型欧拉积分) ===
    public void applyPhysics(double dt) {
        // 1. 更新速度 (v = v0 + a * dt)
        this.speed += this.plannedAcceleration * dt;

        // 2. 更新 Heading (基于自行车模型公式)
        double yawRate = (this.speed / WHEELBASE) * Math.tan(this.plannedSteering);
        this.heading += yawRate * dt;

        // 3. 更新 X 和 Y 坐标 (三角函数分解)
        this.x += this.speed * Math.cos(this.heading) * dt;
        this.y += this.speed * Math.sin(this.heading) * dt;

        // 可选：根据速度和 Heading 顺便更新一下 vx 和 vy，方便外部读取
        this.vx = this.speed * Math.cos(this.heading);
        this.vy = this.speed * Math.sin(this.heading);
        //跨线
        this.lane_index = (int) Math.round(this.y / 4.0);
    }


}
