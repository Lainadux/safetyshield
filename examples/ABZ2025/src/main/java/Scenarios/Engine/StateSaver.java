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

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.lang.reflect.Type;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

public class StateSaver {
    private static final Gson gson = new GsonBuilder()
            .registerTypeAdapter(Vehicle.class, new VehicleAdapter())
            .setPrettyPrinting()
            .create();

    public static void saveState(List<Vehicle> vehicles, String filePath) {
        File outputFile = new File(filePath);
        File parent = outputFile.getParentFile();
        if (parent != null) {
            try {
                Files.createDirectories(parent.toPath());
            } catch (IOException e) {
                throw new UncheckedIOException("Could not create state directory: " + parent.getAbsolutePath(), e);
            }
        }

        try (FileWriter writer = new FileWriter(outputFile)) {
            gson.toJson(vehicles, writer);
            System.out.println("Saved vehicle state: " + outputFile.getAbsolutePath());
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to save vehicle state: " + filePath, e);
        }
    }

    public static List<Vehicle> loadState(String filePath) {
        try (FileReader reader = new FileReader(filePath)) {
            Type listType = new TypeToken<ArrayList<Vehicle>>(){}.getType();
            return gson.fromJson(reader, listType);
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to load vehicle state: " + filePath, e);
        }
    }
}
