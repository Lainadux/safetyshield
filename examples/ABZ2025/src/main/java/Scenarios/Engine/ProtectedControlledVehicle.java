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

public class ProtectedControlledVehicle extends ControlledVehicle {

    public ProtectedControlledVehicle() {
        super();
    }
    public ProtectedControlledVehicle(Vehicle original) {
//        v.id = String.valueOf((int) state.get(offSet + VarTable.id.ordinal()));
//        v.politeness = state.get(offSet + VarTable.politeness.ordinal());
//        v.cooldownTimer = state.get(offSet + VarTable.cooldownTimer.ordinal());
//        v.starked_target_lane_index = state.get(offSet + VarTable.target_lane_index.ordinal());
//        v.starked_lane_index = state.get(offSet + VarTable.lane_index.ordinal());
//        v.x = state.get(offSet + VarTable.x.ordinal());
//        v.y = state.get(offSet + VarTable.y.ordinal());
//        v.vx = state.get(offSet + VarTable.vx.ordinal());
//        v.vy = state.get(offSet + VarTable.vy.ordinal());
//        v.speed = state.get(offSet + VarTable.speed.ordinal());
//        v.heading = state.get(offSet + VarTable.heading.ordinal());
//        v.plannedAcceleration = state.get(offSet + VarTable.plannedAcceleration.ordinal());
//        v.plannedSteering = state.get(offSet + VarTable.plannedSteering.ordinal());
//        v.role = state.get(offSet + VarTable.role.ordinal()) == 0.0 ? "EGO" : "NPC";
//        v.starked = true;
//        v.targetSpeed = state.get(offSet + VarTable.targetSpeed.ordinal());
        this.id = original.id;
        this.politeness = original.politeness;
        //this.starked = original.starked;

        this.setTargetLaneIndex(original.getTargetLaneIndex());
        this.setLaneIndex(original.getLaneIndex());
        this.cooldownTimer = original.cooldownTimer;
        //this.starked_lane_index = original.starked_lane_index;
        //this.starked_target_lane_index = original.starked_target_lane_index;
        this.x = original.x;
        this.y = original.y;
        this.vx = original.vx;
        this.speed = original.speed;
        this.heading = original.heading;
        this.plannedSteering = original.plannedSteering;
        this.plannedAcceleration = original.plannedAcceleration;
        this.role = "EGO";

        this.targetSpeed = original.targetSpeed;


    }

    @Override
    public void signify() {
        //do nothing, the decision is from the real ControlledVehicle, which performed fetchDesiredLaneAndTargetSpeed()
    }
}
