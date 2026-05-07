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

import java.util.List;

public class ControlledVehicle extends Vehicle {

    public ControlledVehicle() {
        super();
        this.role = "EGO";
    }
    public ControlledVehicle(double x, double y, int lane_index, double speed) {
        super();
        this.x = x;
        this.y = y;
        this.lane_index = lane_index;
        this.speed = speed;
        this.target_lane_index = lane_index;
        this.targetSpeed = speed;
        this.role = "EGO";
    }

    public void fetchDesiredLaneAndTargetSpeed() {
        HighwayEngine engine = this.getEngine();

        if(engine.stepCount% engine.STEPS_PER_SECOND == 0 ){
           this.randomActionGenerator();
        }

    }
    public void randomActionGenerator(){

        boolean changeLane = Math.random() < 0.3;
        if(changeLane){
            int[] lanes = this.getEngine().computePossibleLanes(this);

            int newLane = lanes[(int)(Math.random() * lanes.length)];
            this.target_lane_index = newLane;

        }
        else{
            double rand = Math.random();
            if(rand < 0.5){
                //不变速
            }
            else if(rand < 0.85){
                //加速
                this.targetSpeed += 5;
            }
            else{
                //减速
                this.targetSpeed -= 5;
            }
        }
    }
}
