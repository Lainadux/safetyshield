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

public class Main {
    public static void main(String[] args) throws Exception {
        System.out.println("你好");
        //HighwayEngine engine = new ControlledHighwayEngine(0.02);
        HighwayEngine engine = new HighwayEngine(0.02);


        //engine.populateTraffic(4, 3, 0.0, 50.0);
        engine.populateTraffic(4, 2, 0.0, 50.0);


//        Vehicle ego = new Vehicle();
//        ego.role = "NPC";
//        ego.x = 10;
//        ego.y = 0;
//        ego.lane_index = 0;
//        ego.target_lane_index = 0;
//        ego.speed = 20;
//        ego.targetSpeed = 40;
//        ego.id = "debug";
//        engine.addVehicle(ego);
//
//
//        Vehicle npc = new Vehicle();
//        npc.role = "NPC";
//        npc.x = 30;
//        npc.y = 0.0;
//        npc.lane_index = 0;
//        npc.target_lane_index = 0;
//        npc.speed = 15;
//        engine.addVehicle(npc);
//        npc.injectEngine(engine);
//        ego.injectEngine(engine);


        int frameCount = 0;
        while (true) {
            engine.step();
            engine.render();

            frameCount++;
        }
    }
}
