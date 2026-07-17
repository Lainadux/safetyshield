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

package RefractoredVersion.TestScript;

import RefractoredVersion.Engine.AIProfile;
import RefractoredVersion.TestScript.Config.Config;
import RefractoredVersion.TestScript.Config.EgoType;
import RefractoredVersion.TestScript.Config.JavaMomentumConfig;
public class TestScripts {

    public static void main(String[] args) throws Exception {
       // config should be  used to set the parameters of the simulation
       JavaMomentumConfig config = new JavaMomentumConfig();
       config.setQuickTest(true);
       config.setEgoType(EgoType.EgoVehicle);
       config.setAiProfile(AIProfile.adversarial);
       config.setMinX(0);
       config.setMaxX(400);
       config.setDuration(40);
       config.setFrequency(20);

       config.setMaxTargetSpeed(30);
       config.setPredictionTime(1);



       Runner runner = new Runner();
       runner.run(config);
    }
}
