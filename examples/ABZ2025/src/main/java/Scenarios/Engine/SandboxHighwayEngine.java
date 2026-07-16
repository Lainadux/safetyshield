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

public class SandboxHighwayEngine extends HighwayEngine{
    boolean crashed =false;
    boolean crashStateSaved = false;
    boolean egoCollisionOnly = false;
    @Override
    public void checkCollisions() {

        for (int i = 0; i < vehicles.size(); i++) {
            for (int j = i + 1; j < vehicles.size(); j++) {
                Vehicle v1 = vehicles.get(i);
                Vehicle v2 = vehicles.get(j);


                double dx = Math.abs(v1.x - v2.x);
                double dy = Math.abs(v1.y - v2.y);


                boolean overlapX = dx < (v1.LENGTH / 2.0 + v2.LENGTH / 2.0);
                boolean overlapY = dy < (2.0 / 2.0 + 2.0 / 2.0); // 鍋囪 WIDTH 鏄?2.0

                if (overlapX && overlapY) {
                    if (egoCollisionOnly && !isEgoInvolved(v1, v2)) {
                        continue;
                    }

                    String crashMsg = String.format(
                            "馃挜 鑷村懡鐗╃悊纰版挒妫€娴嬭Е鍙戯紒\n" +
                                    "鑲囦簨杞﹁締锛歔%s-%s] 涓?[%s-%s] 鍙戠敓浜嗛噸鍙狅紒\n" +
                                    "鎺ヨЕ鐐瑰潗鏍?-> V1 X:%.1f Y:%.1f | V2 X:%.1f Y:%.1f",
                            v1.getRole(), v1.id,
                            v2.getRole(), v2.id,
                            v1.x, v1.y, v2.x, v2.y
                    );
                    if (requireCollisionLog) {
                        System.err.println(crashMsg);
                        if(!crashStateSaved) {
                            StateSaver.saveState(this.vehicles, "engine " + System.currentTimeMillis() + ".json");
                            crashStateSaved = true;
                        }
                    }
                    //throw new RuntimeException(crashMsg);
                    this.crashed = true;
                }
            }
        }
    }

    private boolean isEgoInvolved(Vehicle v1, Vehicle v2) {
        return isEgoVehicle(v1) || isEgoVehicle(v2);
    }

    private boolean isEgoVehicle(Vehicle vehicle) {
        return vehicle instanceof ControlledVehicle || "EGO".equals(vehicle.role);
    }
}
