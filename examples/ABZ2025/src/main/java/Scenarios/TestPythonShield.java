package Scenarios;

import Scenarios.Engine.PythonMomentumShieldApp;
import py4j.GatewayServer;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.CountDownLatch;

public class TestPythonShield {
    public static void main(String[] args) throws Exception {
        PythonMomentumShieldApp app = new PythonMomentumShieldApp();
        GatewayServer server = new GatewayServer(app);
        server.start();
        System.out.println("PythonMomentumShield GatewayServer started.");
        System.out.println("Python controls prediction_seconds, radius, samples, and randomization.");

        List<String> arguments = new ArrayList<>(Arrays.asList(args));
        if (arguments.remove("--launch-python")) {
            Process process = launchPython(arguments);
            int exitCode = process.waitFor();
            server.shutdown();
            if (exitCode != 0) {
                throw new IllegalStateException("Python momentum shield test failed with exit code " + exitCode);
            }
            return;
        }

        System.out.println("Waiting for Python client. Run python_momentum_shield.py from the Python project.");
        new CountDownLatch(1).await();
    }

    private static Process launchPython(List<String> forwardedArgs) throws IOException {
        Path codesRoot = resolveCodesRoot();
        Path pythonProject = codesRoot.resolve("abz2025_casestudy_autonomous_driving");
        Path script = pythonProject.resolve("python_momentum_shield.py");
        Path python = resolvePythonExecutable(pythonProject);

        List<String> command = new ArrayList<>();
        command.add(python.toString());
        command.add(script.toString());
        command.addAll(forwardedArgs);

        ProcessBuilder builder = new ProcessBuilder(command);
        builder.directory(pythonProject.toFile());
        builder.inheritIO();
        return builder.start();
    }

    private static Path resolvePythonExecutable(Path pythonProject) {
        String configured = System.getProperty("abz.ai.python");
        if (configured == null || configured.isBlank()) {
            configured = System.getenv("ABZ_AI_PYTHON");
        }
        if (configured != null && !configured.isBlank()) {
            return Paths.get(configured);
        }

        Path bundledPython = pythonProject.resolve(Paths.get("env", "Scripts", "python.exe"));
        if (Files.exists(bundledPython)) {
            return bundledPython;
        }
        return Paths.get(System.getProperty("os.name").toLowerCase().contains("win") ? "python" : "python3");
    }

    private static Path resolveCodesRoot() {
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
}
