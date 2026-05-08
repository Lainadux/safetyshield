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

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;
import java.io.*;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;

public class StateSaver {
    // 开启 PrettyPrinting 可以让导出的 JSON 文件有换行和缩进，方便人类阅读
    private static final Gson gson = new GsonBuilder()
            .registerTypeAdapter(Vehicle.class, new VehicleAdapter()) // 注册适配器
            .setPrettyPrinting()
            .create();

    /**
     * 将车辆状态保存到文件
     */
    public static void saveState(List<Vehicle> vehicles, String filePath) {
        try (FileWriter writer = new FileWriter(filePath)) {
            gson.toJson(vehicles, writer);
            System.out.println("💾 场景已保存至: " + filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * 从文件读取车辆状态
     */
    public static List<Vehicle> loadState(String filePath) {
        try (FileReader reader = new FileReader(filePath)) {
            // 🌟 关键：告诉 GSON 这是一组 Vehicle 组成的 List
            Type listType = new TypeToken<ArrayList<Vehicle>>(){}.getType();
            return gson.fromJson(reader, listType);
        } catch (IOException e) {
            System.err.println("❌ 读取场景失败: " + e.getMessage());
            return new ArrayList<>();
        }
    }
}
