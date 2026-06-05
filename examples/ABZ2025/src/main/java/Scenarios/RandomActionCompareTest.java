package Scenarios;

import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.StateSaver;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;

public class RandomActionCompareTest {
    private static final int TEST_COUNT = 100;
    private static final int THREAD_COUNT = 4;
    private static final long RANDOM_ACTION_SEED_BASE = 202605280000L;

    public static void main(String[] args) throws Exception {
        GenerateInitialLogsRun.GenerationConfig config = new GenerateInitialLogsRun.GenerationConfig();
        config.logDir = "examples/ABZ2025/src/main/java/Scenarios/random_action_compare";
        config.comment = "random action generator comparison";
        config.populateTargetVehicles = 18;
        config.populateNumLanes = 3;
        config.populateMinX = 0.0;
        config.populateMaxX = 400.0;
        config.placeEgoAtTrafficMiddle = true;
        config.polite = false;
        config.shieldEgoRangeMeters = 200.0;
        config.checkChangeLaneToRearVehicleThreat = false;
        config.randomizeShieldHiddenTargetAndCooldown = false;

        run(config);
    }

    public static void run(GenerateInitialLogsRun.GenerationConfig config) throws Exception {
        Path logDir = Path.of(config.logDir);
        Path initialStateDir = logDir.resolve("initial_states");
        Files.createDirectories(initialStateDir);

        List<Path> initialStateFiles = generateInitialStates(config, initialStateDir);
        ExperimentResult noShieldResult = runExperiment(config, initialStateFiles,
                "random_action_no_shield", StarkScenarioRunner.DecisionMode.NO_SHIELD, false);
        ExperimentResult starkBaseResult = runExperiment(config, initialStateFiles,
                "random_action_stark_shield_without_rear_threat_check", StarkScenarioRunner.DecisionMode.STARK_SHIELD, false);
        ExperimentResult starkRearThreatResult = runExperiment(config, initialStateFiles,
                "random_action_stark_shield_with_rear_threat_check", StarkScenarioRunner.DecisionMode.STARK_SHIELD, true);

        writeDebugCrashFiles(logDir, starkBaseResult, "CRASH_WITHOUT_REAR_THREAT_");
        writeDebugCrashFiles(logDir, starkRearThreatResult, "CRASH_WITH_REAR_THREAT_");
        writeComparisonFile(logDir, noShieldResult, starkBaseResult, starkRearThreatResult);
    }

    private static List<Path> generateInitialStates(GenerateInitialLogsRun.GenerationConfig config,
                                                   Path initialStateDir) {
        List<Path> files = new ArrayList<>();
        for (int i = 0; i < TEST_COUNT; i++) {
            HighwayEngine realWorld = new HighwayEngine(config.dt, true);
            realWorld.placeEgoAtTrafficMiddle = config.placeEgoAtTrafficMiddle;
            realWorld.egoCentered = config.egoCentered;
            realWorld.npcVehicleType = config.npcVehicleType;
            realWorld.npcIdmActionStepLength = config.npcIdmActionStepLength;
            realWorld.populateTraffic(config.populateTargetVehicles, config.populateNumLanes,
                    config.populateMinX, config.populateMaxX, config.polite);
            realWorld.enhancedCollisionCheckEnabled = config.enhancedCollisionCheckEnabled;

            long seed = RANDOM_ACTION_SEED_BASE + i + 1;
            Path file = initialStateDir.resolve(String.format(Locale.US,
                    "random_initial_%04d_seed_%d.json", i + 1, seed));
            StateSaver.saveState(realWorld.vehicles, file.toString());
            files.add(file);
        }
        return files;
    }

    private static ExperimentResult runExperiment(GenerateInitialLogsRun.GenerationConfig config,
                                                  List<Path> initialStateFiles,
                                                  String methodName,
                                                  StarkScenarioRunner.DecisionMode decisionMode,
                                                  boolean checkChangeLaneToRearVehicleThreat) throws Exception {
        int workerCount = Math.max(1, Math.min(THREAD_COUNT, initialStateFiles.size()));
        ExecutorService executor = Executors.newFixedThreadPool(workerCount);
        List<Future<Path>> futures = new ArrayList<>();
        AtomicInteger completed = new AtomicInteger(0);

        try {
            for (Path initialStateFile : initialStateFiles) {
                futures.add(executor.submit(() -> {
                    HighwayEngine realWorld = StarkScenarioRunner.createWorldFromLog(
                            initialStateFile.toString(),
                            config.dt
                    );

                    StarkScenarioRunner.RunOptions options = new StarkScenarioRunner.RunOptions();
                    options.decisionMode = decisionMode;
                    options.useRandomActionGenerator = true;
                    options.pauseAfterShieldDecision = false;
                    options.saveInitialState = false;
                    options.renderEachStep = false;
                    options.rethrowOnCrash = false;
                    options.printDiagnostics = false;
                    options.timeForSimulationSeconds = config.timeForSimulationSeconds;
                    options.shieldPredictFutureSeconds = config.shieldPredictFutureSeconds;
                    options.shieldEgoRangeMeters = config.shieldEgoRangeMeters;
                    options.randomizeShieldHiddenTargetAndCooldown = config.randomizeShieldHiddenTargetAndCooldown;
                    options.readShieldIdmCooldownTimer = config.readShieldIdmCooldownTimer;
                    options.checkChangeLaneToRearVehicleThreat = checkChangeLaneToRearVehicleThreat;
                    options.aiProfile = config.aiProfile;
                    options.randomActionSeed = stableSeed(initialStateFile);

                    StarkScenarioRunner.RunResult result = StarkScenarioRunner.runScenario(realWorld, options);
                    int done = completed.incrementAndGet();
                    System.out.printf("%s random-action replay %d/%d%n",
                            methodName, done, initialStateFiles.size());
                    return result.crashed ? initialStateFile : null;
                }));
            }

            long crashCount = 0;
            List<Path> crashedFiles = new ArrayList<>();
            for (Future<Path> future : futures) {
                Path crashedFile = future.get();
                if (crashedFile != null) {
                    crashCount++;
                    crashedFiles.add(crashedFile);
                }
            }
            return new ExperimentResult(methodName, initialStateFiles.size(), crashCount, crashedFiles);
        } finally {
            executor.shutdownNow();
        }
    }

    static long stableSeed(Path initialStateFile) {
        Long seedFromName = seedFromFileName(initialStateFile);
        if (seedFromName != null) {
            return seedFromName;
        }
        long seed = 1125899906842597L;
        String key = String.valueOf(initialStateFile.getFileName());
        for (int i = 0; i < key.length(); i++) {
            seed = 31 * seed + key.charAt(i);
        }
        return seed;
    }

    private static Long seedFromFileName(Path initialStateFile) {
        String name = initialStateFile.getFileName().toString();
        int seedStart = name.lastIndexOf("_seed_");
        int jsonEnd = name.lastIndexOf(".json");
        if (seedStart < 0 || jsonEnd <= seedStart) {
            return null;
        }
        try {
            return Long.parseLong(name.substring(seedStart + "_seed_".length(), jsonEnd));
        } catch (NumberFormatException ignored) {
            return null;
        }
    }

    private static void writeDebugCrashFiles(Path logDir, ExperimentResult result, String prefix) throws Exception {
        Path debugDir = logDir.resolve("debug_crashes").resolve(result.methodName);
        Files.createDirectories(debugDir);
        for (Path crashedFile : result.crashedFiles) {
            Path target = debugDir.resolve(prefix + crashedFile.getFileName());
            Files.copy(crashedFile, target, StandardCopyOption.REPLACE_EXISTING);
        }
    }

    private static void writeComparisonFile(Path logDir,
                                            ExperimentResult noShieldResult,
                                            ExperimentResult starkBaseResult,
                                            ExperimentResult starkRearThreatResult) throws Exception {
        Files.createDirectories(logDir);
        double rearThreatCrashProbabilityDelta =
                starkRearThreatResult.crashProbability() - starkBaseResult.crashProbability();
        String markdown = String.format(Locale.US, """
                # Random Action Comparison

                - generatedAt: %s
                - rearThreatCheckCrashProbabilityDelta: %.8f

                | method | totalCount | crashCount | crashProbability |
                | --- | ---: | ---: | ---: |
                | %s | %d | %d | %.8f |
                | %s | %d | %d | %.8f |
                | %s | %d | %d | %.8f |
                """,
                LocalDateTime.now(),
                rearThreatCrashProbabilityDelta,
                noShieldResult.methodName,
                noShieldResult.totalCount,
                noShieldResult.crashCount,
                noShieldResult.crashProbability(),
                starkBaseResult.methodName,
                starkBaseResult.totalCount,
                starkBaseResult.crashCount,
                starkBaseResult.crashProbability(),
                starkRearThreatResult.methodName,
                starkRearThreatResult.totalCount,
                starkRearThreatResult.crashCount,
                starkRearThreatResult.crashProbability()
        );
        Files.writeString(logDir.resolve("random_action_comparison.md"), markdown, StandardCharsets.UTF_8);
    }

    private static class ExperimentResult {
        private final String methodName;
        private final long totalCount;
        private final long crashCount;
        private final List<Path> crashedFiles;

        private ExperimentResult(String methodName, long totalCount, long crashCount, List<Path> crashedFiles) {
            this.methodName = methodName;
            this.totalCount = totalCount;
            this.crashCount = crashCount;
            this.crashedFiles = crashedFiles;
        }

        private double crashProbability() {
            return totalCount == 0 ? 0.0 : (double) crashCount / totalCount;
        }
    }
}
