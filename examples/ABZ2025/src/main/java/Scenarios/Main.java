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

public class Main {
    enum Usage{
        ENGINE,
        STARK,
        RECOVERFROMLOG,
        DEBUG
    }
    public static void main(String[] args) throws Exception {
        Usage usage;
        usage = Usage.ENGINE;
        usage = Usage.RECOVERFROMLOG;
        usage = Usage.STARK;
       // usage = Usage.DEBUG;
        if(usage == Usage.DEBUG){
            HighwayEngine highwayEngine = new HighwayEngine(0.02, true);
            HighwayEngine highwayEngine2 = new HighwayEngine(0.02, true);
            highwayEngine.populateTraffic(6, 3, 0.0, 50.0);
            List<Vehicle> vehicles = highwayEngine.vehicles;
            ControlledVehicle egoVehicle = highwayEngine.getEgoVehicle();
            ProtectedControlledVehicle protectedControlledVehicle = new ProtectedControlledVehicle(egoVehicle);
            protectedControlledVehicle.fetchDesiredLaneAndTargetSpeed(true);

            highwayEngine.setEgoVehicle(protectedControlledVehicle);
            List<Vehicle> vehicles2 = new ArrayList<>();
            for(Vehicle v : vehicles){
                if(v instanceof ControlledVehicle){
                    ControlledVehicle cv =  v.deepCopySelf().ascendAsControlledVehicle();
                    cv.simulated = true;
                    vehicles2.add(cv);
                }
                else{
                    v = v.deepCopySelf();
                    vehicles2.add(v);
                }
            }
            highwayEngine2.vehicles = vehicles2;
            for(Vehicle v : vehicles2){
                v.injectEngine(highwayEngine2);
            }
            highwayEngine.step();
            highwayEngine2.step();
            System.out.println(highwayEngine2.vehicles);
            System.out.println(highwayEngine.vehicles);

        }
        if(usage == Usage.RECOVERFROMLOG){

            List<Vehicle> vehicles = StateSaver.loadState("bug1.json");
            vehicles.set(0, vehicles.get(0).ascendAsControlledVehicle());
            //vehicles.remove(1);
            //vehicles.remove(4);
            //vehicles.remove(0);
            HighwayEngine engine = new HighwayEngine(0.02, true, false, vehicles);
            int frameCount = 0;
            while (true) {
                engine.step();
                engine.render();

                frameCount++;
            }
        }

        if(usage == Usage.STARK) {
            double dt = 0.02;
            HighwayEngine realWorld = new HighwayEngine(dt, true);
            realWorld.populateTraffic(8, 3, 0.0, 50.0);
            ControlledVehicle egoVehicle = realWorld.getEgoVehicle();
            ProtectedControlledVehicle protectedControlledVehicle = new ProtectedControlledVehicle(egoVehicle);
            realWorld.setEgoVehicle(protectedControlledVehicle);

            boolean isSafe = true;

           // while (realWorld.stepCount % realWorld.STEPS_PER_SECOND < realWorld.STEPS_PER_SECOND - 1) {
            while (realWorld.stepCount  <=  0) {
                //when the ego has taken an decision
                if(realWorld.stepCount % realWorld.STEPS_PER_SECOND == 0){
                    protectedControlledVehicle.fetchDesiredLaneAndTargetSpeed();
                    System.out.println("real world scenario");
                    for(Vehicle v: realWorld.vehicles){
                        System.out.println(v);
                    }
                }
               //testing simulation
                if(realWorld.stepCount == 0){
                    //egoVehicle.fetchDesiredLaneAndTargetSpeed();

                    System.out.println("Step: " + realWorld.stepCount + ", Time: " + realWorld.runTime);
                    List<Vehicle> vehicles = new ArrayList<>();
                    for(Vehicle v: realWorld.vehicles){
                        if(v instanceof ControlledVehicle){
                            ControlledVehicle cv =  v.deepCopySelf().ascendAsControlledVehicle();
                            cv.simulated = true;
                            vehicles.add(cv);
                        }
                        else{
                            v = v.deepCopySelf();
                            vehicles.add(v);
                        }
                    }
                    HighwayEngine engine = new HighwayEngine(dt, true, true, vehicles);
                    StarkShieldApp starkShieldApp = engine.createStarkShieldApp();
                }


                if (isSafe) {
                    realWorld.step();
                } else {
                    realWorld.step(protectedControlledVehicle.lane_index, protectedControlledVehicle.targetSpeed<0?0:protectedControlledVehicle.targetSpeed-5);
                }
                realWorld.render();


            }
            System.out.println("end state of real world scenario");
            for(Vehicle v: realWorld.vehicles){
                System.out.println(v);
            }
        }
        //HighwayEngine engine = new ControlledHighwayEngine(0.02);
        if(usage == Usage.ENGINE) {
            HighwayEngine engine = new HighwayEngine(0.02, true);
            //engine.requireCollisionLog = true;
            engine.enhancedCollisionCheckEnabled = true;
            engine.saveInitStateAnyWay = true;
            engine.egoCentered = true;

            //engine.populateTraffic(4, 3, 0.0, 50.0);
            engine.populateTraffic(6, 3, 0.0, 50.0);
            int frameCount = 0;
            while (true) {
                engine.step();
                engine.render();

                frameCount++;
            }


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
        }



    }

    public static List<Vehicle> fetchInitialVehicles(double dt, int numVehicles, int numLanes, double minX, double maxX){
        HighwayEngine highwayEngine = new HighwayEngine(dt);
        highwayEngine.populateTraffic(numVehicles, numLanes, minX, maxX);
        return highwayEngine.vehicles;

    }

}
