package Scenarios;

import Scenarios.Engine.HighwayAiClient;
import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.StarkShieldApp;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;

public class GenerateInitialLogsRun {
    private static final int LOG_COUNT = 100;
    private static final int THREAD_COUNT = 4;

    public static void main(String[] args, boolean polite, String logSuffixID) throws Exception {
        main(args, polite, logSuffixID, "");
    }

    public static void main(String[] args, boolean polite, String logSuffixID, String comment) throws Exception {
        GenerationConfig config = new GenerationConfig();
        config.polite = polite;
        config.logDir = "examples/ABZ2025/src/main/java/Scenarios/logs" + logSuffixID;
        config.comment = comment;
        run(config);
    }

    public static void run(GenerationConfig config) throws Exception {
        writeCommentFile(config);

        int workerCount = Math.max(1, Math.min(THREAD_COUNT, LOG_COUNT));
        ExecutorService executor = Executors.newFixedThreadPool(workerCount);
        List<Future<?>> futures = new ArrayList<>();
        AtomicInteger completed = new AtomicInteger(0);
        StarkScenarioRunner.VerifyTimingStats verifyTimingStats = new StarkScenarioRunner.VerifyTimingStats();

        try {
            for (int i = 0; i < LOG_COUNT; i++) {
                final int logIndex = i + 1;
                futures.add(executor.submit(() -> {
                    System.out.printf("Generating log %d/%d%n", logIndex, LOG_COUNT);
                    HighwayEngine realWorld = createWorld(config);

                    StarkScenarioRunner.RunOptions options = new StarkScenarioRunner.RunOptions();
                    options.pauseAfterShieldDecision = false;
                    options.saveInitialState = true;
                    options.renderEachStep = false;
                    options.rethrowOnCrash = false;
                    options.printDiagnostics = false;
                    options.logDir = config.logDir;
                    options.timeForSimulationSeconds = config.timeForSimulationSeconds;
                    options.shieldPredictFutureSeconds = config.shieldPredictFutureSeconds;
                    options.shieldEgoRangeMeters = config.shieldEgoRangeMeters;
                    options.randomizeShieldHiddenTargetAndCooldown = config.randomizeShieldHiddenTargetAndCooldown;
                    options.readShieldIdmCooldownTimer = config.readShieldIdmCooldownTimer;
                    options.checkChangeLaneToRearVehicleThreat = config.checkChangeLaneToRearVehicleThreat;
                    options.aiProfile = config.aiProfile;
                    options.verifyTimingStats = verifyTimingStats;

                    StarkScenarioRunner.runShieldedScenario(realWorld, options);
                    int done = completed.incrementAndGet();
                    System.out.printf("Finished log %d/%d%n", done, LOG_COUNT);
                    return null;
                }));
            }

            for (Future<?> future : futures) {
                future.get();
            }
        } finally {
            writeVerifyTimingFile(config, verifyTimingStats);
            executor.shutdownNow();
            HighwayAiClient.stopAll();
        }
    }

    private static HighwayEngine createWorld(GenerationConfig config) {
        HighwayEngine realWorld = new HighwayEngine(config.dt, true);
        realWorld.placeEgoAtTrafficMiddle = config.placeEgoAtTrafficMiddle;
        realWorld.egoCentered = config.egoCentered;
        realWorld.npcVehicleType = config.npcVehicleType;
        realWorld.npcIdmActionStepLength = config.npcIdmActionStepLength;
        realWorld.populateTraffic(config.populateTargetVehicles, config.populateNumLanes,
                config.populateMinX, config.populateMaxX, config.polite);
        realWorld.enhancedCollisionCheckEnabled = config.enhancedCollisionCheckEnabled;
        return realWorld;
    }

    private static void writeCommentFile(GenerationConfig config) throws IOException {
        Path logPath = Path.of(config.logDir);
        Files.createDirectories(logPath);
        String markdown = buildCommentMarkdown(config);
        Files.writeString(logPath.resolve("comment.md"), markdown, StandardCharsets.UTF_8);
    }

    private static void writeVerifyTimingFile(GenerationConfig config,
                                              StarkScenarioRunner.VerifyTimingStats verifyTimingStats) throws IOException {
        Path logPath = Path.of(config.logDir);
        Files.createDirectories(logPath);
        StarkScenarioRunner.VerifyTimingStats.Snapshot snapshot = verifyTimingStats.snapshot();
        String markdown = String.format("""
                # StarkShield Verify Timing

                - generatedAt: %s
                - sampleCount: %d
                - rejectedCount: %d
                - rejectProbability: %.6f
                - meanMs: %.3f
                - varianceMsSquared: %.3f
                - stdMs: %.3f
                - minMs: %.3f
                - maxMs: %.3f

                ## Reject Probability By Ego Speed

                | egoSpeedRange(m/s) | sampleCount | rejectedCount | rejectProbability |
                | --- | ---: | ---: | ---: |
                %s
                """,
                LocalDateTime.now(),
                snapshot.count,
                snapshot.rejectedCount,
                snapshot.rejectProbability(),
                snapshot.meanMillis(),
                snapshot.varianceMillisSquared(),
                snapshot.stdMillis(),
                snapshot.minMillis(),
                snapshot.maxMillis(),
                buildSpeedRejectDistributionMarkdown(snapshot)
        );
        Files.writeString(logPath.resolve("verify_timing.md"), markdown, StandardCharsets.UTF_8);
    }

    private static String buildSpeedRejectDistributionMarkdown(StarkScenarioRunner.VerifyTimingStats.Snapshot snapshot) {
        if (snapshot.speedRejectBuckets.isEmpty()) {
            return "| none | 0 | 0 | 0.000000 |";
        }

        StringBuilder builder = new StringBuilder();
        for (StarkScenarioRunner.VerifyTimingStats.SpeedRejectBucketSnapshot bucket : snapshot.speedRejectBuckets) {
            builder.append(String.format("| [%.1f, %.1f) | %d | %d | %.6f |%n",
                    bucket.minSpeed,
                    bucket.maxSpeed,
                    bucket.count,
                    bucket.rejectedCount,
                    bucket.rejectProbability()));
        }
        return builder.toString();
    }

    private static String buildCommentMarkdown(GenerationConfig config) {
        return String.format("""
                # Generate Initial Logs

                ## Comment

                %s

                ## Configuration

                - generatedAt: %s
                - logDir: `%s`
                - logCount: %d
                - threadCount: %d
                - dt: %.3f
                - simulationSeconds: %d
                - shieldPredictFutureSeconds: %d
                - aiProfile: `%s`
                - realWorldPopulateMethod: `HighwayEngine.populateTraffic(int targetVehicles, int numLanes, double minX, double maxX, boolean polite)`
                - realWorldPopulateArguments: `targetVehicles=%d, numLanes=%d, minX=%.1f, maxX=%.1f, polite=%s`
                - placeEgoAtTrafficMiddle: %s
                - egoCentered: %s
                - initialStateDescription: one EGO vehicle is spawned first with `id=0`, `role=EGO`; its x is either `0` or `(minX+maxX)/2` according to `placeEgoAtTrafficMiddle`; when `egoCentered=true`, its lane is the middle lane; remaining vehicles are NPCs sampled in the configured lane/x range.
                - initialSpeedDistribution: `speed = clippedGaussian(mean=20, std=3, range=[10,30])`
                - initialTargetSpeedDistribution: `targetSpeed = speed + uniform(0,5)`
                - initialCooldownDistribution: `cooldownTimer = uniform(0,1)`
                - initialSpawnSafety: rejected and resampled when the initial state is not dynamically safe according to `HighwayEngine.isSpawnDynamicallySafe`
                - realWorldNpcVehicleType: `%s`
                - realWorldNpcIdmActionStepLength: %.3f
                - realWorldPolitenessRandomized: %s
                - starkShieldRadius: vehicles within +/- %.1f meters of EGO are visible to StarkShield
                - starkShieldTargetSpeedSource: %s
                - starkShieldCooldownTimerSource: %s
                - starkShieldIdmCooldownTimerSource: %s
                - checkChangeLaneToRearVehicleThreat: %s
                """,
                normalizeComment(config.comment),
                LocalDateTime.now(),
                config.logDir,
                LOG_COUNT,
                THREAD_COUNT,
                config.dt,
                config.timeForSimulationSeconds,
                config.shieldPredictFutureSeconds,
                config.aiProfile,
                config.populateTargetVehicles,
                config.populateNumLanes,
                config.populateMinX,
                config.populateMaxX,
                config.polite,
                config.placeEgoAtTrafficMiddle,
                config.egoCentered,
                config.npcVehicleType,
                config.npcIdmActionStepLength,
                config.polite ? "true, NPC politeness is sampled from clipped Gaussian mean=0.5 std=1/6 range=[0,1]"
                        : "false, NPC politeness remains 0.0",
                config.shieldEgoRangeMeters,
                config.randomizeShieldHiddenTargetAndCooldown
                        ? "randomized for NPCs, clipped Gaussian mean=30 std=10/3 range=[20,40]; ego uses real targetSpeed"
                        : "passed from real world vehicle state",
                config.randomizeShieldHiddenTargetAndCooldown
                        ? "randomized for NPCs, clipped Gaussian mean=0.5 std=1/6 range=[0,1]; ego uses real cooldownTimer"
                        : "passed from real world vehicle state",
                config.readShieldIdmCooldownTimer
                        ? "passed from real world vehicle state"
                        : "randomized for IDM cooldown NPCs, uniform range=[0,idmActionStepLength]",
                config.checkChangeLaneToRearVehicleThreat
        );
    }

    private static String normalizeComment(String comment) {
        if (comment == null || comment.isBlank()) {
            return "(empty)";
        }
        return comment;
    }

    public static class GenerationConfig {
        public String logDir = StarkScenarioRunner.DEFAULT_LOG_DIR;
        public String comment = "";
        public double dt = StarkScenarioRunner.DEFAULT_DT;
        public int timeForSimulationSeconds = StarkScenarioRunner.DEFAULT_TIME_FOR_SIMULATION_SECONDS;
        public int shieldPredictFutureSeconds = 3;
        public double shieldEgoRangeMeters = StarkShieldApp.DEFAULT_SHIELD_EGO_RANGE_METERS;
        public boolean randomizeShieldHiddenTargetAndCooldown = StarkShieldApp.DEFAULT_RANDOMIZE_HIDDEN_TARGET_AND_COOLDOWN;
        public boolean readShieldIdmCooldownTimer = StarkShieldApp.DEFAULT_READ_SHIELD_IDM_COOLDOWN_TIMER;
        public boolean checkChangeLaneToRearVehicleThreat = false;
        public String aiProfile = HighwayAiClient.getConfiguredAiProfile();
        public int populateTargetVehicles = 36;
        public int populateNumLanes = 3;
        public double populateMinX = 0.0;
        public double populateMaxX = 400.0;
        public boolean placeEgoAtTrafficMiddle = false;
        public boolean egoCentered = false;
        public boolean polite = false;
        public boolean enhancedCollisionCheckEnabled = true;
        public HighwayEngine.NpcVehicleType npcVehicleType = HighwayEngine.NpcVehicleType.DEFAULT;
        public double npcIdmActionStepLength = 0.1;
    }
}
