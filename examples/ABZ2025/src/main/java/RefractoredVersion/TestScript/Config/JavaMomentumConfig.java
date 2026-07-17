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

package RefractoredVersion.TestScript.Config;

import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class JavaMomentumConfig extends Config {
    private Integer duration = DEFAULT_DURATION;
    private Integer stepsToPredict = DEFAULT_STEPS_TO_PREDICT;
    private Integer numOFSimulations = DEFAULT_NUM_OF_SIMULATIONS;
    private MethodToGenInitialState methodToGenInitialState = MethodToGenInitialState.DEFAULT;
    private EgoType egoType = EgoType.RandomEgoVehicle;
    private int frequency = 40;
    private int predictionTime = 3;
    private double maxTargetSpeed = 40.0;
    private boolean fixPrediction = false;
    private double minX;
    private double maxX;
    // when quickTest, it shows one rendered random run
    private boolean quickTest;





}
