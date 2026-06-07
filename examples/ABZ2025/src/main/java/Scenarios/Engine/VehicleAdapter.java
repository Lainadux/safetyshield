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

import com.google.gson.Gson;
import com.google.gson.JsonDeserializationContext;
import com.google.gson.JsonDeserializer;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import com.google.gson.JsonSerializationContext;
import com.google.gson.JsonSerializer;

import java.lang.reflect.Type;

public class VehicleAdapter implements JsonDeserializer<Vehicle>, JsonSerializer<Vehicle> {
    private static final Gson pureGson = new Gson();

    @Override
    public JsonElement serialize(Vehicle src, Type typeOfSrc, JsonSerializationContext context) {
        JsonObject result = pureGson.toJsonTree(src).getAsJsonObject();
        result.addProperty("vehicle_type", src.getClass().getSimpleName());
        return result;
    }

    @Override
    public Vehicle deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context) throws JsonParseException {
        JsonObject jsonObject = json.getAsJsonObject();
        JsonElement typeElement = jsonObject.get("vehicle_type");

        if (typeElement != null) {
            String type = typeElement.getAsString();
            if ("ProtectedControlledVehicle".equals(type)) {
                return pureGson.fromJson(json, ProtectedControlledVehicle.class);
            }
            if ("InstantProtectedControlledVehicle".equals(type)) {
                return pureGson.fromJson(json, InstantProtectedControlledVehicle.class);
            }
            if ("ControlledVehicle".equals(type)) {
                return pureGson.fromJson(json, ControlledVehicle.class);
            }
            if ("IDMCooldownVehicle".equals(type)) {
                return pureGson.fromJson(json, IDMCooldownVehicle.class);
            }
            if ("DelayedIDMVehicle".equals(type)) {
                return pureGson.fromJson(json, DelayedIDMVehicle.class);
            }
        }
        if (jsonObject.has("reactionDelay")) {
            return pureGson.fromJson(json, DelayedIDMVehicle.class);
        }
        if (jsonObject.has("idmActionStepLength") || jsonObject.has("idmCooldownTimer")) {
            return pureGson.fromJson(json, IDMCooldownVehicle.class);
        }

        return pureGson.fromJson(json, Vehicle.class);
    }
}
