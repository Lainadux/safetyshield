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

import java.util.concurrent.ThreadLocalRandom;

public class RandomEgoVehicle extends Vehicle implements NonNpcVehicle, NotControlledByMOBIL, PControlledVehicle {

    @Override
    public void planAction() throws Exception {
        if (this.getEngine().isDecisionTime()){
            Action randomAction = Action.fromValue(ThreadLocalRandom.current().nextInt(5));
            switch (randomAction) {
                case FASTER:
                    this.targetSpeed += 5;
                    break;
                case SLOWER:
                    this.targetSpeed = Math.max(0.0, this.targetSpeed - 5);
                    break;
                case IDLE:
                    break;
                case LANE_LEFT:
                    this.setTargetLaneIndex(Math.max(0, this.getTargetLaneIndex() - 1));
                    break;
                case LANE_RIGHT:
                    this.setTargetLaneIndex(Math.min(2, this.getTargetLaneIndex() + 1));
                    break;
                default:
                    break;
            }

        }
        else{

        }

        this.plannedAcceleration = JavaHighwayEngineUtils.computeIdmAcceleration(this, this.getEngine().vehicles);
        this.plannedSteering = JavaHighwayEngineUtils.computeSteering(this);
    }
}
