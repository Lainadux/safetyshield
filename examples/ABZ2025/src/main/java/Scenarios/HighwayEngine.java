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

import java.util.ArrayList;
import java.util.List;

public class HighwayEngine {
    public List<Vehicle> vehicles;
    public double dt;
    public HighwayEngine(double dt) {
        this.vehicles = new ArrayList<>();
        this.dt = dt;
    }

    public void addVehicle(Vehicle v) {
        this.vehicles.add(v);
    }
    public void step() throws Exception {
        // 【第一阶段：全局冻结，各自思考】
        // 所有车看着彼此当前的（旧）位置，决定下一秒干嘛
        for (Vehicle v : vehicles) {
            v.planAction(vehicles);
        }

        // 【第二阶段：时间流逝，集体结算】
        // 所有车同时应用刚才的决定，更新坐标
        for (Vehicle v : vehicles) {
            v.applyPhysics(dt);
        }
    }


}

