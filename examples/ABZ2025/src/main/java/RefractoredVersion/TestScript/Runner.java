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

import RefractoredVersion.Engine.JavaHighwayEngine;
import RefractoredVersion.Engine.Vehicle;
import RefractoredVersion.Engine.VehicleGenerator;
import RefractoredVersion.TestScript.Config.Config;
import RefractoredVersion.TestScript.Config.FallBackMode;
import RefractoredVersion.TestScript.Config.JavaMomentumConfig;
import java.util.List;

public class Runner {
    Config config;
    public void run(Config config) throws Exception {
        this.config = config;
        if(config instanceof JavaMomentumConfig){
            JavaMomentumConfig javaMomentumConfig = (JavaMomentumConfig) config;
            if(javaMomentumConfig.isQuickTest()){
                singleRun(true);
                return;
            }

            for(int i =0; i<javaMomentumConfig.getDuration();i++){
                if(javaMomentumConfig.saveInitial) throw new UnsupportedOperationException("saveInitial not implemented yet");
                throw new UnsupportedOperationException("saveInitial not implemented yet");
//                Monitor m = new Monitor();
//
//                RealWorld r = new RealWorld();
//
//                for(int i=0; i<r.simulationTime;i)

            }
        }
    }

    public void singleRun(boolean rendered) throws Exception {
        if(config instanceof JavaMomentumConfig){
            JavaMomentumConfig javaMomentumConfig = (JavaMomentumConfig) config;
            JavaHighwayEngine realWorld = new JavaHighwayEngine();
            realWorld.setFrequency(javaMomentumConfig.getFrequency());
            realWorld.config = javaMomentumConfig;
            List<Vehicle> vehicles = VehicleGenerator.generateVehicles(javaMomentumConfig);
            realWorld.fallBackMode= FallBackMode.DEFAULT;
            realWorld.vehicles = vehicles;

            for(int i =0; i<javaMomentumConfig.getDuration()* javaMomentumConfig.getFrequency();i++){

                realWorld.step();
                if(rendered){
                    realWorld.render();
                }
            }


        }


    }
}
