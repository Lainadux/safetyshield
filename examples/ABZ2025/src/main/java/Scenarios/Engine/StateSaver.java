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
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;
import java.io.*;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;

public class StateSaver {
    // 寮€鍚?PrettyPrinting 鍙互璁╁鍑虹殑 JSON 鏂囦欢鏈夋崲琛屽拰缂╄繘锛屾柟渚夸汉绫婚槄璇?
    private static final Gson gson = new GsonBuilder()
            .registerTypeAdapter(Vehicle.class, new VehicleAdapter()) // 娉ㄥ唽閫傞厤鍣?
            .setPrettyPrinting()
            .create();

    /**
     * 灏嗚溅杈嗙姸鎬佷繚瀛樺埌鏂囦欢
     */
    public static void saveState(List<Vehicle> vehicles, String filePath) {
        try (FileWriter writer = new FileWriter(filePath)) {
            gson.toJson(vehicles, writer);
            System.out.println("馃捑 鍦烘櫙宸蹭繚瀛樿嚦: " + filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * 浠庢枃浠惰鍙栬溅杈嗙姸鎬?
     */
    public static List<Vehicle> loadState(String filePath) {
        try (FileReader reader = new FileReader(filePath)) {
            // 馃専 鍏抽敭锛氬憡璇?GSON 杩欐槸涓€缁?Vehicle 缁勬垚鐨?List
            Type listType = new TypeToken<ArrayList<Vehicle>>(){}.getType();
            return gson.fromJson(reader, listType);
        } catch (IOException e) {
            System.err.println("鉂?璇诲彇鍦烘櫙澶辫触: " + e.getMessage());
            return new ArrayList<>();
        }
    }
}
