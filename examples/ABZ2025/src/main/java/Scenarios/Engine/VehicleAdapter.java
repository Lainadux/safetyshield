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

import com.google.gson.*;
import java.lang.reflect.Type;

import com.google.gson.*;
import java.lang.reflect.Type;

public class VehicleAdapter implements JsonDeserializer<Vehicle>, JsonSerializer<Vehicle> {

    // 馃専 鏍稿績鐮村眬鐐癸細鍒涘缓涓€涓病鏈変换浣曡嚜瀹氫箟閫傞厤鍣ㄧ殑鈥滅函鍑€鐗?Gson鈥?
    private static final Gson pureGson = new Gson();

    @Override
    public JsonElement serialize(Vehicle src, Type typeOfSrc, JsonSerializationContext context) {
        // 浣跨敤 pureGson 搴忓垪鍖栵紝闃叉鏃犻檺閫掑綊
        JsonObject result = pureGson.toJsonTree(src).getAsJsonObject();
        // 鎵撲笂绫诲瀷鎬濇兂閽㈠嵃
        result.addProperty("vehicle_type", src.getClass().getSimpleName());
        return result;
    }

    @Override
    public Vehicle deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context) throws JsonParseException {
        JsonObject jsonObject = json.getAsJsonObject();
        JsonElement typeElement = jsonObject.get("vehicle_type");

        if (typeElement != null) {
            String type = typeElement.getAsString();

            // 馃専 浣跨敤 pureGson 鍙嶅簭鍒楀寲鍏蜂綋瀛愮被锛屽畠浼氳嚜鍔ㄥ皢 JSON 濉叆 ControlledVehicle 鐨勫瓧娈典腑
            if ("ControlledVehicle".equals(type)) {
                return pureGson.fromJson(json, ControlledVehicle.class);
            }
            // 濡傛灉鏈潵鏈?IDMVehicle, 鍙互鍦ㄨ繖閲屽姞 else if ("IDMVehicle".equals(type)) ...
        }

        // 馃専 榛樿鎯呭喌锛氫娇鐢?pureGson 鍙嶅簭鍒楀寲涓哄熀绫?
        return pureGson.fromJson(json, Vehicle.class);
    }
}