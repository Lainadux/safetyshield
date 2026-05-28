package Scenarios.Engine;

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
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.TimeUnit;

public class HighwayAiClient {
    private static final String DEFAULT_AI_PROFILE = "base";
    //private static final String DEFAULT_AI_PROFILE = "adversarial";
    private static final Gson GSON = new Gson();
    private static final CopyOnWriteArrayList<HighwayAiClient> CLIENTS = new CopyOnWriteArrayList<>();
    private static final ThreadLocal<HighwayAiClient> INSTANCE = ThreadLocal.withInitial(() -> {
        HighwayAiClient client = new HighwayAiClient();
        CLIENTS.add(client);
        return client;
    });

    private Process process;
    private BufferedWriter input;
    private BufferedReader output;

    private HighwayAiClient() {
    }

    public static HighwayAiClient getInstance() {
        return INSTANCE.get();
    }

    public static void stopAll() {
        for (HighwayAiClient client : CLIENTS) {
            client.stop();
        }
        CLIENTS.clear();
    }

    public synchronized AiDecision decide(ControlledVehicle ego) throws IOException {
        ensureStarted();

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

    private void ensureStarted() throws IOException {
        if (process != null && process.isAlive()) {
            return;
        }

        Path python = resolvePythonExecutable();
        Path script = resolveAiScript();
        ProcessBuilder builder = new ProcessBuilder(
                python.toString(),
                script.toString(),
                "ai-server",
                getConfiguredAiProfile()
        );
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
        System.out.println("AI server ready: " + ready);
    }

    private List<Map<String, Object>> toVehiclePayload(ControlledVehicle ego) {
        HighwayEngine engine = ego.getEngine();
        List<Map<String, Object>> vehicles = new ArrayList<>();
        for (Vehicle vehicle : engine.vehicles) {
            String role = vehicle == ego ? "EGO" : "NPC";
            vehicles.add(Map.of(
                    "role", role,
                    "x", vehicle.x,
                    "y", vehicle.y,
                    "vx", vehicle.vx != 0.0 ? vehicle.vx : vehicle.speed,
                    "vy", vehicle.vy,
                    "speed", vehicle.speed,
                    "heading", vehicle.heading
            ));
        }
        return vehicles;
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
        if (Files.exists(bundledPython) && isVenvBasePythonAvailable(projectRoot.resolve("env").resolve("pyvenv.cfg"))) {
            return bundledPython;
        }
        if (System.getProperty("os.name").toLowerCase().contains("win")) {
            return Paths.get("python");
        }
        return Paths.get("python");
    }

    private boolean isVenvBasePythonAvailable(Path pyvenvConfig) {
        if (!Files.exists(pyvenvConfig)) {
            return true;
        }
        try {
            for (String line : Files.readAllLines(pyvenvConfig, StandardCharsets.UTF_8)) {
                if (line.startsWith("executable = ")) {
                    return Files.exists(Paths.get(line.substring("executable = ".length()).trim()));
                }
            }
        } catch (IOException ignored) {
            return true;
        }
        return true;
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

    public static String getConfiguredAiProfile() {
        String configured = System.getProperty("abz.ai.profile");
        if (configured == null || configured.isBlank()) {
            configured = System.getenv("ABZ_AI_PROFILE");
        }
        if (configured == null || configured.isBlank()) {
            return DEFAULT_AI_PROFILE;
        }
        return configured;
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
    }

    public static class AiDecision {
        public int action = 1;
        public String action_name = "IDLE";
        public String error;
    }
}
