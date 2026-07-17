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

import com.google.gson.Gson;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class JavaHighwayAiClient {
    private static final Gson GSON = new Gson();
    private static final JavaHighwayAiClient INSTANCE = new JavaHighwayAiClient();

    private Process process;
    private BufferedWriter input;
    private BufferedReader output;
    private String activeProfile;

    private JavaHighwayAiClient() {
    }

    public static JavaHighwayAiClient getInstance() {
        return INSTANCE;
    }

    public synchronized AiDecision decide(EgoVehicle ego) throws IOException {
        ensureStarted(ego.aiProfile);

        String request = GSON.toJson(Map.of("vehicles", toVehiclePayload(ego)));
        input.write(request);
        input.newLine();
        input.flush();

        String response = output.readLine();
        if (response == null) {
            stop();
            throw new IOException("AI server stopped without returning a decision");
        }

        AiDecision decision = GSON.fromJson(response, AiDecision.class);
        if (decision == null) {
            throw new IOException("AI server returned an empty decision");
        }
        if (decision.error != null && !decision.error.isBlank()) {
            throw new IOException("AI server error: " + decision.error);
        }
        return decision;
    }

    private void ensureStarted(AIProfile profile) throws IOException {
        String requestedProfile = profile == null ? AIProfile.base.name() : profile.name();
        if (process != null
                && process.isAlive()
                && requestedProfile.equals(activeProfile)) {
            return;
        }
        stop();

        List<String> command = new ArrayList<>();
        command.add(resolvePythonExecutable().toString());
        Path script = resolveAiScript();
        command.add(script.toString());
        command.add("ai-server");
        command.add(requestedProfile);

        ProcessBuilder builder = new ProcessBuilder(command);
        builder.directory(script.getParent().toFile());
        builder.redirectError(ProcessBuilder.Redirect.INHERIT);

        process = builder.start();
        input = new BufferedWriter(new OutputStreamWriter(process.getOutputStream(), StandardCharsets.UTF_8));
        output = new BufferedReader(new InputStreamReader(process.getInputStream(), StandardCharsets.UTF_8));

        String ready = output.readLine();
        if (ready == null || !ready.contains("\"ready\"")) {
            stop();
            throw new IOException("AI server did not become ready");
        }
        activeProfile = requestedProfile;
        System.out.println("AI server ready: " + ready);
    }

    private List<Map<String, Object>> toVehiclePayload(EgoVehicle ego) {
        List<Map<String, Object>> payload = new ArrayList<>();
        for (Vehicle vehicle : ego.getDetectedVehicles()) {
            payload.add(Map.of(
                    "role", vehicle == ego ? "EGO" : "NPC",
                    "x", vehicle.x,
                    "y", vehicle.y,
                    "vx", vehicle.vx != 0.0 ? vehicle.vx : vehicle.speed,
                    "vy", vehicle.vy,
                    "speed", vehicle.speed,
                    "heading", vehicle.heading
            ));
        }
        return payload;
    }

    private Path resolvePythonExecutable() {
        String configured = System.getProperty("abz.ai.python");
        if (configured == null || configured.isBlank()) {
            configured = System.getenv("ABZ_AI_PYTHON");
        }
        if (configured != null && !configured.isBlank()) {
            return Paths.get(configured);
        }

        Path projectRoot = resolveCodesRoot().resolve("abz2025_casestudy_autonomous_driving");
        Path bundledPython = projectRoot.resolve(Paths.get("env", "Scripts", "python.exe"));
        if (Files.exists(bundledPython)) {
            return bundledPython;
        }
        return Paths.get("python");
    }

    private Path resolveAiScript() {
        String configured = System.getProperty("abz.ai.script");
        if (configured == null || configured.isBlank()) {
            configured = System.getenv("ABZ_AI_SCRIPT");
        }
        if (configured != null && !configured.isBlank()) {
            return Paths.get(configured);
        }
        return resolveCodesRoot()
                .resolve("abz2025_casestudy_autonomous_driving")
                .resolve("HighwayEnvironment_Base.py");
    }

    private Path resolveCodesRoot() {
        Path cwd = Paths.get(System.getProperty("user.dir")).toAbsolutePath();
        Path candidate = cwd;
        while (candidate != null) {
            if (Files.isDirectory(candidate.resolve("abz2025_casestudy_autonomous_driving"))) {
                return candidate;
            }
            candidate = candidate.getParent();
        }
        return Paths.get("C:\\MSCProject\\codes");
    }

    private void stop() {
        try {
            if (input != null) {
                input.close();
            }
        } catch (IOException ignored) {
        }
        if (process != null) {
            process.destroy();
            try {
                if (!process.waitFor(1, TimeUnit.SECONDS)) {
                    process.destroyForcibly();
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                process.destroyForcibly();
            }
        }
        process = null;
        input = null;
        output = null;
        activeProfile = null;
    }

    public static class AiDecision {
        public int action = Action.IDLE.getValue();
        public String action_name = Action.IDLE.name();
        public String error;
    }
}
