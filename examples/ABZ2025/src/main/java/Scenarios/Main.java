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

import Scenarios.Engine.*;
import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;

public class Main {
    enum Usage{
        ENGINE,
        STARK,
        RECOVERFROMLOG,
        DEBUG
    }
    public static void main(String[] args) throws Exception {
        //System.err.println("---------");
        Usage usage;
        usage = Usage.ENGINE;
        usage = Usage.RECOVERFROMLOG;
        usage = Usage.STARK;
       // usage = Usage.DEBUG;
        if(usage == Usage.DEBUG){
            HighwayEngine highwayEngine = new HighwayEngine(0.02, true);
            HighwayEngine highwayEngine2 = new HighwayEngine(0.02, true);
            highwayEngine.populateTraffic(6, 3, 0.0, 100.0);
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
            realWorld.enhancedCollisionCheckEnabled = true;
            realWorld.populateTraffic(7, 3, 0.0, 80.0);
            ControlledVehicle egoVehicle = realWorld.getEgoVehicle();
            ProtectedControlledVehicle protectedControlledVehicle = new ProtectedControlledVehicle(egoVehicle);
            realWorld.setEgoVehicle(protectedControlledVehicle);

            boolean isSafe = true;
            int timeForSimulation = 40;
            double prevTgtspd = 0.0;
            int prevCurrentLane = 0;
            while (realWorld.stepCount != timeForSimulation * realWorld.STEPS_PER_SECOND - 1) {

            //while (realWorld.stepCount  <=  0) {
                //when the ego has taken an decision
                if(realWorld.stepCount % realWorld.STEPS_PER_SECOND == 0){
                    prevTgtspd = protectedControlledVehicle.targetSpeed;
                    prevCurrentLane = protectedControlledVehicle.getLaneIndex();

                    protectedControlledVehicle.fetchDesiredLaneAndTargetSpeed();
//                    System.out.println("real world scenario");
//                    for(Vehicle v: realWorld.vehicles){
//                        System.out.println(v);
//                    }
                }
               //testing simulation
//                if(realWorld.stepCount == 0){
//                    //egoVehicle.fetchDesiredLaneAndTargetSpeed();
//                    System.out.println("real world scenario：");
//                    for(Vehicle v: realWorld.vehicles){
//                        System.out.println(v);
//                    }
//                    List<Vehicle> vehicles = new ArrayList<>();
//                    for(Vehicle v: realWorld.vehicles){
//                        if(v instanceof ControlledVehicle){
//                            ControlledVehicle cv =  v.deepCopySelf().ascendAsControlledVehicle();
//                            cv.simulated = true;
//                            vehicles.add(cv);
//                        }
//                        else{
//                            v = v.deepCopySelf();
//                            vehicles.add(v);
//                        }
//                    }
//                    HighwayEngine engine = new HighwayEngine(dt, true, true, vehicles);
//                    StarkShieldApp starkShieldApp = engine.createStarkShieldApp(10);
//                }
                //test shield

                if(realWorld.stepCount % realWorld.STEPS_PER_SECOND == 0){
                    //egoVehicle.fetchDesiredLaneAndTargetSpeed();
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
                    StarkShieldApp starkShieldApp = engine.createStarkShieldApp(4);
                    isSafe = starkShieldApp.verifySafe();
                    if(!isSafe){
                        HighwayAiClient.AiDecision unsafeDecision = protectedControlledVehicle.getLastAiDecision();
                        System.out.printf("Unsafe AI decision: action=%d, action_name=%s%n",
                                unsafeDecision.action, unsafeDecision.action_name);
                        System.out.println(starkShieldApp.getUnsafeDiagnosis());
                        protectedControlledVehicle.targetSpeed = prevTgtspd -5 <0? 0: prevTgtspd -5;
                        protectedControlledVehicle.setTargetLaneIndex(prevCurrentLane);
                        waitForSpaceToContinue(realWorld);

                    }
                }
                realWorld.step();
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

    private static void waitForSpaceToContinue(HighwayEngine engine) throws InterruptedException {
        System.out.println("Unsafe AI decision detected. Simulation paused. Press SPACE in the simulator window to continue.");
        engine.render();

        CountDownLatch spacePressed = new CountDownLatch(1);
        KeyEventDispatcher dispatcher = event -> {
            if (event.getID() == KeyEvent.KEY_PRESSED && event.getKeyCode() == KeyEvent.VK_SPACE) {
                spacePressed.countDown();
                return true;
            }
            return false;
        };

        KeyboardFocusManager focusManager = KeyboardFocusManager.getCurrentKeyboardFocusManager();
        focusManager.addKeyEventDispatcher(dispatcher);
        try {
            spacePressed.await();
        } finally {
            focusManager.removeKeyEventDispatcher(dispatcher);
        }
    }

    public static List<Vehicle> fetchInitialVehicles(double dt, int numVehicles, int numLanes, double minX, double maxX){
        HighwayEngine highwayEngine = new HighwayEngine(dt);
        highwayEngine.populateTraffic(numVehicles, numLanes, minX, maxX);
        return highwayEngine.vehicles;

    }

}
