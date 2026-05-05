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

import py4j.GatewayServer;
import com.google.gson.Gson;

// 1. 写一个普通的 Java 类，提供一个验证方法
public class ShieldForTwoLanes {
    private Gson gson = new Gson();
    // 这个方法接收 Python 发来的字符串，返回一个布尔值给 Python
    public boolean verifySafety(String action, String vehiclesJson) {
        System.out.println("receing action: " + action);
        // 这里调用你 STARK 的核心逻辑
        VehicleState[] vehicles = gson.fromJson(vehiclesJson, VehicleState[].class);

//        OneLane oneLane = new OneLane(vehicles, action);
//        oneLane.initialize(vehicles, action);
//        boolean safe = oneLane.verify();

        TwoLane twoLanes = new TwoLane(vehicles, action);
//        twoLanes.initialize(vehicles, action);
        boolean safe = twoLanes.verify();


        //boolean isSafe = true; //
        return safe; // 直接 return！Py4J 会负责把它送回给 Python
    }

    public static void main(String[] args) {
        ShieldForTwoLanes app = new ShieldForTwoLanes(); // 创建一个实例
        // 2. 启动 GatewayServer，把 app 暴露给 Python
        GatewayServer server = new GatewayServer(app);
        server.start();
        System.out.println("🛡️ STARK Gateway Server 已启动！");
    }
}