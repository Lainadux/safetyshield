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

package Scenarios.Engine;

import java.util.List;
import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

public class ControlledVehicle extends Vehicle {
    public boolean simulated = false;
    private HighwayAiClient.AiDecision lastAiDecision = new HighwayAiClient.AiDecision();

    public ControlledVehicle() {
        super();
        this.role = "EGO";
    }
    public ControlledVehicle(Vehicle vehicle) {
        super(vehicle);
        this.role = "EGO";

        //copy all attributes

    }
    public ControlledVehicle(double x, double y, int lane_index, double speed, double targetSpeed) {

        super();
        this.id = "0";
        this.x = x;
        this.y = y;
//        this.lane_index = lane_index;
        this.setLaneIndex(lane_index);
        this.speed = speed;
        this.setTargetLaneIndex(lane_index);
        this.targetSpeed = targetSpeed;
        this.role = "EGO";
    }

    public void performShieldedAction(){

    }
    public void signify(){
        if(!simulated){
            this.fetchDesiredLaneAndTargetSpeed(); //fetch AI decision
        }
        else{
            HighwayEngine engine = this.getEngine();
            if(engine.stepCount% engine.STEPS_PER_SECOND == 0 ){
                //do nothing, the decision is from the real ControlledVehicle, which performed fetchDesiredLaneAndTargetSpeed()
            }
            else{
                //protection logic锛?deceleration
                this.target_lane_index = this.getTargetLaneIndex();
                this.targetSpeed = this.targetSpeed -5 < 0 ? 0 : this.targetSpeed -5;

            }

        }

    }
    public ControlledVehicle cloneForSimulation() {
        ControlledVehicle clone = new ControlledVehicle(this.x, this.y, this.getLaneIndex(), this.speed, this.targetSpeed);
        clone.setTargetLaneIndex(this.getTargetLaneIndex());
        clone.targetSpeed = this.targetSpeed;
        clone.simulated = true;
        return clone;
    }
    public void fetchDesiredLaneAndTargetSpeed() {
        HighwayEngine engine = this.getEngine();

        if(engine.stepCount% engine.STEPS_PER_SECOND == 0 ){
            try {
                this.applyAiAction(HighwayAiClient.getInstance().decide(this));
            } catch (Exception e) {
                System.err.println("AI decision failed, falling back to random action: " + e.getMessage());
                this.randomActionGenerator();
            }
        }

    }
    public void fetchDesiredLaneAndTargetSpeed(boolean random) throws Exception {
        if(random) this.applyAiAction(randomAiDecision());
        else{
            this.applyAiAction(HighwayAiClient.getInstance().decide(this));

        }

    }

    public void fetchRandomDesiredLaneAndTargetSpeed() {
        HighwayEngine engine = this.getEngine();
        if(engine.stepCount % engine.STEPS_PER_SECOND == 0) {
            this.applyAiAction(randomAiDecision());
        }
    }

    public void applyRandomDecision(Random random) {
        this.applyAiAction(randomAiDecision(random));
    }

    public static HighwayAiClient.AiDecision randomAiDecision() {
        return randomAiDecision(ThreadLocalRandom.current().nextInt(5));
    }

    public static HighwayAiClient.AiDecision randomAiDecision(Random random) {
        return randomAiDecision(random.nextInt(5));
    }

    private static HighwayAiClient.AiDecision randomAiDecision(int action) {
        HighwayAiClient.AiDecision decision = new HighwayAiClient.AiDecision();
        decision.action = action;
        decision.action_name = switch (action) {
            case 0 -> "LANE_LEFT";
            case 2 -> "LANE_RIGHT";
            case 3 -> "FASTER";
            case 4 -> "SLOWER";
            default -> "IDLE";
        };
        return decision;
    }
    protected void applyAiAction(HighwayAiClient.AiDecision decision) {
        this.lastAiDecision = decision;
        String action = decision.action_name == null ? "" : decision.action_name.trim().toUpperCase();
        if (action.isEmpty()) {
            action = switch (decision.action) {
                case 0 -> "LANE_LEFT";
                case 2 -> "LANE_RIGHT";
                case 3 -> "FASTER";
                case 4 -> "SLOWER";
                default -> "IDLE";
            };
        }

        switch (action) {
            case "LANE_LEFT" -> this.setTargetLaneIndex(clampLane(this.getLaneIndex() - 1));
            case "LANE_RIGHT" -> this.setTargetLaneIndex(clampLane(this.getLaneIndex() + 1));
            case "FASTER" -> this.targetSpeed += 5;
            case "SLOWER" -> this.targetSpeed = Math.max(0, this.targetSpeed - 5);
            default -> {
            }
        }
    }

    private int clampLane(int lane) {
        HighwayEngine engine = this.getEngine();
        return Math.max(0, Math.min(engine.numLanes - 1, lane));
    }

    public HighwayAiClient.AiDecision getLastAiDecision() {
        return lastAiDecision;
    }

    public void randomActionGenerator(){
        if (ThreadLocalRandom.current().nextInt(1) == 0) {
            this.applyAiAction(randomAiDecision());
            return;
        }

        boolean changeLane = Math.random() < 0.3;
        if(changeLane){
            int[] lanes = this.getEngine().computePossibleLanes(this);

            int newLane = lanes[(int)(Math.random() * lanes.length)];
            //this.target_lane_index = newLane;
            //this.setLaneIndex(newLane);
            this.setTargetLaneIndex(newLane);

        }
        else{
            double rand = Math.random();
            if(rand < 0.2){
                //涓嶅彉閫?
            }
            else if(rand < 1){
                //鍔犻€?
                this.targetSpeed += 5;
            }
            else{
                //鍑忛€?
                this.targetSpeed = this.targetSpeed <= 5 ? 0 : this.targetSpeed - 5;

            }
        }
    }
}
