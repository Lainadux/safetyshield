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

import com.google.gson.*;
import java.lang.reflect.Type;

import com.google.gson.*;
import java.lang.reflect.Type;

public class VehicleAdapter implements JsonDeserializer<Vehicle>, JsonSerializer<Vehicle> {

    // 🌟 核心破局点：创建一个没有任何自定义适配器的“纯净版 Gson”
    private static final Gson pureGson = new Gson();

    @Override
    public JsonElement serialize(Vehicle src, Type typeOfSrc, JsonSerializationContext context) {
        // 使用 pureGson 序列化，防止无限递归
        JsonObject result = pureGson.toJsonTree(src).getAsJsonObject();
        // 打上类型思想钢印
        result.addProperty("vehicle_type", src.getClass().getSimpleName());
        return result;
    }

    @Override
    public Vehicle deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context) throws JsonParseException {
        JsonObject jsonObject = json.getAsJsonObject();
        JsonElement typeElement = jsonObject.get("vehicle_type");

        if (typeElement != null) {
            String type = typeElement.getAsString();

            // 🌟 使用 pureGson 反序列化具体子类，它会自动将 JSON 填入 ControlledVehicle 的字段中
            if ("ControlledVehicle".equals(type)) {
                return pureGson.fromJson(json, ControlledVehicle.class);
            }
            // 如果未来有 IDMVehicle, 可以在这里加 else if ("IDMVehicle".equals(type)) ...
        }

        // 🌟 默认情况：使用 pureGson 反序列化为基类
        return pureGson.fromJson(json, Vehicle.class);
    }
}