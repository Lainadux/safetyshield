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

public class SandboxJavaHighwayEngine extends JavaHighwayEngine {
    public boolean hasCollision = false;
    public boolean hasNpcCollision = false;
    public boolean hasNonNpcVehicleCollision = false;

    @Override
    public void checkCollision() {
        if (vehicles == null) {
            return;
        }

        for (int i = 0; i < vehicles.size(); i++) {
            for (int j = i + 1; j < vehicles.size(); j++) {
                Vehicle first = vehicles.get(i);
                Vehicle second = vehicles.get(j);
                if (!isColliding(first, second)) {
                    continue;
                }

                hasCollision = true;
                if (first instanceof NonNpcVehicle || second instanceof NonNpcVehicle) {
                    hasNonNpcVehicleCollision = true;
                } else {
                    hasNpcCollision = true;
                }
            }
        }
    }

    private boolean isColliding(Vehicle first, Vehicle second) {
        double dx = Math.abs(first.x - second.x);
        double dy = Math.abs(first.y - second.y);
        return dx < (first.LENGTH + second.LENGTH) / 2.0
                && dy < (first.WIDTH + second.WIDTH) / 2.0;
    }
}
