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

import RefractoredVersion.Shield.AllSlowerShield;

import java.util.ArrayList;

public class EgoVehicle extends Vehicle implements NonNpcVehicle, NotControlledByMOBIL, PControlledVehicle{
    public AIProfile aiProfile = AIProfile.base;
    public int sensorRange = 100;
    public ArrayList<Vehicle> detectedVehicles = new ArrayList<>();
    private JavaHighwayAiClient.AiDecision lastAiDecision = new JavaHighwayAiClient.AiDecision();

    public ArrayList<Vehicle> getDetectedVehicles() {
        detectedVehicles.clear();
        for (Vehicle vehicle : this.getEngine().vehicles) {
            if (vehicle == this) {
                detectedVehicles.add(vehicle);
                continue;
            }

            double longitudinalDistance = vehicle.x - this.x;
            if (Math.abs(longitudinalDistance) <= sensorRange) {
                detectedVehicles.add(vehicle);
            }
        }
        return detectedVehicles;
    }


    @Override
    public void planAction() throws Exception {
        if (this.getEngine().isDecisionTime()){
            JavaHighwayAiClient.AiDecision decision = JavaHighwayAiClient.getInstance().decide(this);
            System.out.printf("AI decision: action=%d, action_name=%s%n",
                    decision.action,
                    decision.action_name);
            applyAiAction(decision);
        }

        this.plannedAcceleration = JavaHighwayEngineUtils.computeIdmAcceleration(this, this.getEngine().vehicles);
        this.plannedSteering = JavaHighwayEngineUtils.computeSteering(this);
    }

    private void applyAiAction(JavaHighwayAiClient.AiDecision decision) throws Exception {
        this.lastAiDecision = decision;
        Action action = parseAction(decision);
        AllSlowerShield shield = new AllSlowerShield(this.getEngine());
        boolean safe = shield.verifySafe();
        System.out.printf("%s AI decision: action=%d, action_name=%s, parsed_action=%s%n",
                safe ? "Safe" : "Unsafe",
                decision.action,
                decision.action_name,
                action);
        if (!safe) {
            System.out.println(shield.getUnsafeDiagnosis());
            action = Action.SLOWER;
        }
        applyAction(action);
    }

    private void applyAction(Action action) {
        switch (action) {
            case LANE_LEFT:
                this.setTargetLaneIndex(Math.max(0, this.getLaneIndex() - 1));
                break;
            case LANE_RIGHT:
                this.setTargetLaneIndex(Math.min(this.getEngine().numLanes - 1, this.getLaneIndex() + 1));
                break;
            case FASTER:
                this.targetSpeed = clipTargetSpeed(this.targetSpeed + 5);
                break;
            case SLOWER:
                this.targetSpeed = clipTargetSpeed(this.targetSpeed - 5);
                break;
            case IDLE:
                break;
            default:
                throw new IllegalArgumentException("Unsupported AI action: " + action);
        }
    }

    private Action parseAction(JavaHighwayAiClient.AiDecision decision) {
        if (decision.action_name != null && !decision.action_name.isBlank()) {
            return Action.valueOf(decision.action_name.trim().toUpperCase());
        }
        return Action.fromValue(decision.action);
    }

    public JavaHighwayAiClient.AiDecision getLastAiDecision() {
        return lastAiDecision;
    }

    private double clipTargetSpeed(double targetSpeed) {
        return Math.max(0.0, Math.min(maxTargetSpeed(), targetSpeed));
    }

    private double maxTargetSpeed() {
        return this.getEngine().config == null ? 40.0 : this.getEngine().config.getMaxTargetSpeed();
    }

    @Override
    public void checkCollision() {
        super.checkCollision();
    }
}
