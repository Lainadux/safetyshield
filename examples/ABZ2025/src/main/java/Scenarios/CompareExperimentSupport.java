package Scenarios;

import Scenarios.Engine.HighwayAiClient;
import Scenarios.Engine.HighwayEngine;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

final class CompareExperimentSupport {
    private static final int THREAD_COUNT = 4;
    private static final Pattern REJECT_PROBABILITY_PATTERN =
            Pattern.compile("- rejectProbability: ([0-9.]+)");
    private static final Pattern SPEED_BUCKET_PATTERN =
            Pattern.compile("\\| \\[([0-9.]+), ([0-9.]+)\\) \\| (\\d+) \\| (\\d+) \\| ([0-9.]+) \\|");

    private CompareExperimentSupport() {
    }

    static TimingProfile readTimingProfile(Path logDir) throws IOException {
        Path timingFile = logDir.resolve("verify_timing.md");
        String content = Files.readString(timingFile, StandardCharsets.UTF_8);

        Matcher rejectMatcher = REJECT_PROBABILITY_PATTERN.matcher(content);
        if (!rejectMatcher.find()) {
            throw new IllegalStateException("Cannot find rejectProbability in " + timingFile);
        }

        TimingProfile profile = new TimingProfile(Double.parseDouble(rejectMatcher.group(1)));
        Matcher bucketMatcher = SPEED_BUCKET_PATTERN.matcher(content);
        while (bucketMatcher.find()) {
            profile.speedBuckets.add(new SpeedBucket(
                    Double.parseDouble(bucketMatcher.group(1)),
                    Double.parseDouble(bucketMatcher.group(2)),
                    Double.parseDouble(bucketMatcher.group(5))
            ));
        }
        profile.speedBuckets.sort(Comparator.comparingDouble(bucket -> bucket.minSpeed));
        return profile;
    }

    static List<Path> listInitialStateFiles(Path logDir) throws IOException {
        try (Stream<Path> stream = Files.list(logDir)) {
            return stream
                    .filter(path -> {
                        String name = path.getFileName().toString();
                        return name.startsWith("safe_initial_") || name.startsWith("crash_initial_");
                    })
                    .filter(path -> path.getFileName().toString().endsWith(".json"))
                    .sorted()
                    .collect(Collectors.toList());
        }
    }

    static ExperimentResult getStarkResult(Path logDir) throws IOException {
        List<Path> files = listInitialStateFiles(logDir);
        long crashCount = files.stream()
                .filter(path -> path.getFileName().toString().startsWith("crash_initial_"))
                .count();
        return new ExperimentResult("stark_shield", files.size(), crashCount);
    }

    static ExperimentResult runProbabilityExperiment(GenerateInitialLogsRun.GenerationConfig config,
                                                     String methodName,
                                                     StarkScenarioRunner.DecisionMode decisionMode,
                                                     TimingProfile timingProfile) throws Exception {
        return runExperiment(config, methodName, decisionMode, timingProfile);
    }

    static ExperimentResult runExperiment(GenerateInitialLogsRun.GenerationConfig config,
                                          String methodName,
                                          StarkScenarioRunner.DecisionMode decisionMode,
                                          TimingProfile timingProfile) throws Exception {
        Path logDir = Path.of(config.logDir);
        List<Path> initialStateFiles = listInitialStateFiles(logDir);
        int workerCount = Math.max(1, Math.min(THREAD_COUNT, initialStateFiles.size()));
        ExecutorService executor = Executors.newFixedThreadPool(workerCount);
        List<Future<Boolean>> futures = new ArrayList<>();
        AtomicInteger completed = new AtomicInteger(0);

        try {
            for (Path initialStateFile : initialStateFiles) {
                futures.add(executor.submit(() -> {
                    HighwayEngine realWorld = StarkScenarioRunner.createWorldFromLog(
                            initialStateFile.toString(),
                            config.dt
                    );

                    StarkScenarioRunner.RunOptions options = new StarkScenarioRunner.RunOptions();
                    options.pauseAfterShieldDecision = false;
                    options.saveInitialState = false;
                    options.renderEachStep = false;
                    options.rethrowOnCrash = false;
                    options.printDiagnostics = false;
                    options.timeForSimulationSeconds = config.timeForSimulationSeconds;
                    options.shieldPredictFutureSeconds = config.shieldPredictFutureSeconds;
                    options.shieldEgoRangeMeters = config.shieldEgoRangeMeters;
                    options.randomizeShieldHiddenTargetAndCooldown = config.randomizeShieldHiddenTargetAndCooldown;
                    options.checkChangeLaneToRearVehicleThreat = config.checkChangeLaneToRearVehicleThreat;
                    options.decisionMode = decisionMode;
                    options.pureRejectProbability = timingProfile == null ? 0.0 : timingProfile.rejectProbability;
                    options.speedRejectProbabilityModel = timingProfile;

                    StarkScenarioRunner.RunResult result = StarkScenarioRunner.runScenario(realWorld, options);
                    int done = completed.incrementAndGet();
                    System.out.printf("%s comparison replay %d/%d%n", methodName, done, initialStateFiles.size());
                    return result.crashed;
                }));
            }

            long crashCount = 0;
            for (Future<Boolean> future : futures) {
                if (future.get()) {
                    crashCount++;
                }
            }
            ExperimentResult result = new ExperimentResult(methodName, initialStateFiles.size(), crashCount);
            writeResult(logDir, result);
            writeComparison(logDir);
            return result;
        } finally {
            executor.shutdownNow();
            HighwayAiClient.stopAll();
        }
    }

    private static void writeResult(Path logDir, ExperimentResult result) throws IOException {
        Files.createDirectories(logDir);
        Files.writeString(logDir.resolve(result.methodName + "_comparison_result.txt"),
                String.format(Locale.US, "%s,%d,%d,%.8f%n",
                        result.methodName,
                        result.totalCount,
                        result.crashCount,
                        result.crashProbability()),
                StandardCharsets.UTF_8);
    }

    static void writeComparison(Path logDir) throws IOException {
        ExperimentResult stark = getStarkResult(logDir);
        Optional<ExperimentResult> noShield = readResult(logDir, "no_shield");
        Optional<ExperimentResult> pure = readResult(logDir, "pure_probability");
        Optional<ExperimentResult> joint = readResult(logDir, "joint_speed_probability");

        String markdown = String.format(Locale.US, """
                # Shield Comparison

                - generatedAt: %s
                - initialStateDir: `%s`

                | method | totalCount | crashCount | crashProbability |
                | --- | ---: | ---: | ---: |
                | stark_shield | %d | %d | %.8f |
                | no_shield | %s | %s | %s |
                | pure_probability | %s | %s | %s |
                | joint_speed_probability | %s | %s | %s |
                """,
                LocalDateTime.now(),
                logDir,
                stark.totalCount,
                stark.crashCount,
                stark.crashProbability(),
                field(noShield, r -> String.valueOf(r.totalCount)),
                field(noShield, r -> String.valueOf(r.crashCount)),
                field(noShield, r -> String.format(Locale.US, "%.8f", r.crashProbability())),
                field(pure, r -> String.valueOf(r.totalCount)),
                field(pure, r -> String.valueOf(r.crashCount)),
                field(pure, r -> String.format(Locale.US, "%.8f", r.crashProbability())),
                field(joint, r -> String.valueOf(r.totalCount)),
                field(joint, r -> String.valueOf(r.crashCount)),
                field(joint, r -> String.format(Locale.US, "%.8f", r.crashProbability()))
        );
        Files.writeString(logDir.resolve("shield_comparison.md"), markdown, StandardCharsets.UTF_8);
    }

    private static Optional<ExperimentResult> readResult(Path logDir, String methodName) throws IOException {
        Path resultFile = logDir.resolve(methodName + "_comparison_result.txt");
        if (!Files.exists(resultFile)) {
            return Optional.empty();
        }
        String[] parts = Files.readString(resultFile, StandardCharsets.UTF_8).trim().split(",");
        if (parts.length < 4) {
            return Optional.empty();
        }
        return Optional.of(new ExperimentResult(parts[0], Long.parseLong(parts[1]), Long.parseLong(parts[2])));
    }

    private static String field(Optional<ExperimentResult> result,
                                java.util.function.Function<ExperimentResult, String> getter) {
        return result.map(getter).orElse("pending");
    }

    static class TimingProfile implements StarkScenarioRunner.SpeedRejectProbabilityModel {
        private final double rejectProbability;
        private final List<SpeedBucket> speedBuckets = new ArrayList<>();

        private TimingProfile(double rejectProbability) {
            this.rejectProbability = rejectProbability;
        }

        double rejectProbability() {
            return rejectProbability;
        }

        @Override
        public double probabilityForSpeed(double egoSpeed) {
            for (SpeedBucket bucket : speedBuckets) {
                if (egoSpeed >= bucket.minSpeed && egoSpeed < bucket.maxSpeed) {
                    return bucket.rejectProbability;
                }
            }
            return rejectProbability;
        }
    }

    private static class SpeedBucket {
        private final double minSpeed;
        private final double maxSpeed;
        private final double rejectProbability;

        private SpeedBucket(double minSpeed, double maxSpeed, double rejectProbability) {
            this.minSpeed = minSpeed;
            this.maxSpeed = maxSpeed;
            this.rejectProbability = rejectProbability;
        }
    }

    static class ExperimentResult {
        private final String methodName;
        private final long totalCount;
        private final long crashCount;

        private ExperimentResult(String methodName, long totalCount, long crashCount) {
            this.methodName = methodName;
            this.totalCount = totalCount;
            this.crashCount = crashCount;
        }

        private double crashProbability() {
            return totalCount == 0 ? 0.0 : (double) crashCount / totalCount;
        }
    }
}
