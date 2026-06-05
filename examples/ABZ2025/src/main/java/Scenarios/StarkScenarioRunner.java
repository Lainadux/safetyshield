package Scenarios;

import Scenarios.Engine.ControlledVehicle;
import Scenarios.Engine.HighwayAiClient;
import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.ProtectedControlledVehicle;
import Scenarios.Engine.StarkShieldApp;
import Scenarios.Engine.StateSaver;
import Scenarios.Engine.Vehicle;

import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;
import java.io.File;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicInteger;
import javax.swing.JOptionPane;

public final class StarkScenarioRunner {
    public static final double DEFAULT_DT = 0.02;
    public static final int DEFAULT_TIME_FOR_SIMULATION_SECONDS = 40;
    public static final String DEFAULT_LOG_DIR = "examples/ABZ2025/src/main/java/Scenarios/logs4";

    private static final DateTimeFormatter STATE_TIME_FORMAT = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS");
    private static final AtomicInteger SAVED_STATE_SEQUENCE = new AtomicInteger(0);

    private StarkScenarioRunner() {
    }

    public static HighwayEngine createRandomWorld(double dt) {
        HighwayEngine realWorld = new HighwayEngine(dt, true);
        //realWorld.populateTraffic(18, 3, 0.0, 200);
        realWorld.populateTraffic(36, 3, 0.0, 400);
        realWorld.enhancedCollisionCheckEnabled = true;
        return realWorld;
    }
    public static HighwayEngine createRandomWorld(double dt, boolean polite) {
        HighwayEngine realWorld = new HighwayEngine(dt, true);
        //realWorld.populateTraffic(18, 3, 0.0, 200);
        realWorld.populateTraffic(36, 3, 0.0, 400, polite);
        realWorld.enhancedCollisionCheckEnabled = true;
        return realWorld;
    }

    public static HighwayEngine createWorldFromLog(String logFile, double dt) {
        List<Vehicle> vehicles = normalizeLoadedVehicles(StateSaver.loadState(logFile));
        HighwayEngine realWorld = new HighwayEngine(dt, true, true, vehicles);
        realWorld.numLanes = inferNumLanes(vehicles);
        realWorld.enhancedCollisionCheckEnabled = true;
        System.out.println("Loaded initial state: " + new File(logFile).getAbsolutePath());
        return realWorld;
    }

    public static void runShieldedScenario(HighwayEngine realWorld, RunOptions options) throws Exception {
        runScenario(realWorld, options);
    }

    public static RunResult runScenario(HighwayEngine realWorld, RunOptions options) throws Exception {
        double dt = realWorld.dt;
        ProtectedControlledVehicle protectedControlledVehicle = ensureProtectedEgo(realWorld);
        List<Vehicle> initialScenario = deepCopyVehicles(realWorld.vehicles);
        Random randomActionGenerator = options.randomActionSeed == null
                ? null
                : new Random(options.randomActionSeed);
        Random probabilityRejectionRandom = options.probabilityRejectionSeed == null
                ? null
                : new Random(options.probabilityRejectionSeed);

        boolean crashed = false;
        boolean initialScenarioSaved = false;
        double prevTgtspd = 0.0;
        int prevCurrentLane = 0;
        long lastStep = (long) options.timeForSimulationSeconds * realWorld.STEPS_PER_SECOND - 1;

        try {
            HighwayAiClient.setThreadAiProfile(options.aiProfile);
            while (realWorld.stepCount != lastStep) {
                if (realWorld.stepCount % realWorld.STEPS_PER_SECOND == 0) {
                    prevTgtspd = protectedControlledVehicle.targetSpeed;
                    prevCurrentLane = protectedControlledVehicle.getLaneIndex();
                    if (options.useRandomActionGenerator) {
                        if (randomActionGenerator == null) {
                            protectedControlledVehicle.fetchRandomDesiredLaneAndTargetSpeed();
                        } else {
                            protectedControlledVehicle.applyRandomDecision(randomActionGenerator);
                        }
                    } else {
                        protectedControlledVehicle.fetchDesiredLaneAndTargetSpeed();
                    }
                }

                if (realWorld.stepCount % realWorld.STEPS_PER_SECOND == 0) {
                    StarkShieldApp starkShieldApp = null;
                    boolean isSafe;
                    if (options.decisionMode == DecisionMode.STARK_SHIELD) {
                        List<Vehicle> shieldVehicles = buildShieldVehicles(realWorld.vehicles);
                        HighwayEngine shieldEngine = new HighwayEngine(dt, true, true, shieldVehicles);
                        starkShieldApp = shieldEngine.createStarkShieldApp(
                                options.shieldPredictFutureSeconds,
                                options.shieldEgoRangeMeters,
                                options.randomizeShieldHiddenTargetAndCooldown,
                                options.checkChangeLaneToRearVehicleThreat,
                                options.readShieldIdmCooldownTimer);
                        long verifyStartNanos = System.nanoTime();
                        isSafe = starkShieldApp.verifySafe();
                        long verifyElapsedNanos = System.nanoTime() - verifyStartNanos;
                        if (options.verifyTimingStats != null) {
                            options.verifyTimingStats.record(verifyElapsedNanos, !isSafe, protectedControlledVehicle.speed);
                        }
                    } else if (options.decisionMode == DecisionMode.NO_SHIELD) {
                        isSafe = true;
                    } else {
                        isSafe = !options.shouldRejectByProbability(protectedControlledVehicle.speed, probabilityRejectionRandom);
                    }
                    HighwayAiClient.AiDecision decision = protectedControlledVehicle.getLastAiDecision();

                    if (options.printDiagnostics) {
                        System.out.printf("%s AI decision: action=%d, action_name=%s%n",
                                isSafe ? "Safe" : "Unsafe", decision.action, decision.action_name);
                        if (starkShieldApp != null) {
                            System.out.println(starkShieldApp.getUnsafeDiagnosis());
                        }
                    }

                    if (!isSafe) {
                        protectedControlledVehicle.targetSpeed = prevTgtspd - 5 < 0 ? 0 : prevTgtspd - 5;
                        protectedControlledVehicle.setTargetLaneIndex(prevCurrentLane);
                    }

                    if (options.pauseAfterShieldDecision) {
                        waitForSpaceToContinue(realWorld, starkShieldApp, options);
                    }
                }

                realWorld.step();
                if (options.renderEachStep) {
                    realWorld.render();
                }
            }
        } catch (RuntimeException e) {
            crashed = true;
            if (options.saveInitialState) {
                saveInitialScenarioSnapshot(initialScenario, true, options.logDir);
            }
            initialScenarioSaved = true;
            if (options.rethrowOnCrash) {
                throw e;
            }
            System.err.println("Scenario crashed, saved crash initial state and continuing: " + e.getMessage());
        } finally {
            HighwayAiClient.setThreadAiProfile(null);
            if (options.saveInitialState && !initialScenarioSaved) {
                saveInitialScenarioSnapshot(initialScenario, crashed, options.logDir);
            }
        }

        if (options.printDiagnostics) {
            System.out.println("end state of real world scenario");
            for (Vehicle v : realWorld.vehicles) {
                System.out.println(v);
            }
        }
        return new RunResult(crashed);
    }

    public static List<Vehicle> normalizeLoadedVehicles(List<Vehicle> loadedVehicles) {
        List<Vehicle> vehicles = new ArrayList<>();
        boolean hasControlledEgo = false;
        for (Vehicle vehicle : loadedVehicles) {
            if (vehicle instanceof ProtectedControlledVehicle) {
                vehicles.add(new ProtectedControlledVehicle(vehicle));
                hasControlledEgo = true;
            } else if (vehicle instanceof ControlledVehicle || "EGO".equals(vehicle.role) || "0".equals(vehicle.id)) {
                vehicles.add(new ProtectedControlledVehicle(vehicle));
                hasControlledEgo = true;
            } else {
                vehicles.add(vehicle.deepCopySelf());
            }
        }
        if (!hasControlledEgo && !vehicles.isEmpty()) {
            vehicles.set(0, new ProtectedControlledVehicle(vehicles.get(0)));
        }
        return vehicles;
    }

    public static int inferNumLanes(List<Vehicle> vehicles) {
        int maxLane = 0;
        for (Vehicle vehicle : vehicles) {
            maxLane = Math.max(maxLane, Math.max(vehicle.getLaneIndex(), vehicle.getTargetLaneIndex()));
        }
        return Math.max(1, maxLane + 1);
    }

    private static ProtectedControlledVehicle ensureProtectedEgo(HighwayEngine realWorld) {
        ControlledVehicle egoVehicle = realWorld.getEgoVehicle();
        if (egoVehicle instanceof ProtectedControlledVehicle) {
            return (ProtectedControlledVehicle) egoVehicle;
        }

        ProtectedControlledVehicle protectedControlledVehicle = new ProtectedControlledVehicle(egoVehicle);
        realWorld.setEgoVehicle(protectedControlledVehicle);
        return protectedControlledVehicle;
    }

    private static List<Vehicle> buildShieldVehicles(List<Vehicle> realWorldVehicles) {
        List<Vehicle> vehicles = new ArrayList<>();
        for (Vehicle v : realWorldVehicles) {
            if (v instanceof ControlledVehicle) {
                ControlledVehicle cv = v.deepCopySelf().ascendAsControlledVehicle();
                cv.simulated = true;
                vehicles.add(cv);
            } else {
                vehicles.add(v.deepCopySelf());
            }
        }
        return vehicles;
    }

    private static List<Vehicle> deepCopyVehicles(List<Vehicle> vehicles) {
        List<Vehicle> copies = new ArrayList<>();
        for (Vehicle vehicle : vehicles) {
            if (vehicle instanceof ProtectedControlledVehicle) {
                copies.add(new ProtectedControlledVehicle(vehicle));
            } else if (vehicle instanceof ControlledVehicle) {
                copies.add(new ControlledVehicle(vehicle));
            } else {
                copies.add(vehicle.deepCopySelf());
            }
        }
        return copies;
    }

    private static void saveInitialScenarioSnapshot(List<Vehicle> initialScenario, boolean crashed, String logDir) {
        String prefix = crashed ? "crash_initial_" : "safe_initial_";
        String timestamp = LocalDateTime.now().format(STATE_TIME_FORMAT);
        int sequence = SAVED_STATE_SEQUENCE.incrementAndGet();
        StateSaver.saveState(initialScenario, logDir + File.separator + prefix + timestamp + "_" + sequence + ".json");
    }

    private static void waitForSpaceToContinue(HighwayEngine engine, StarkShieldApp starkShieldApp, RunOptions options) throws InterruptedException {
        engine.render();
        engine.setPredictedStateQueryAction(() -> promptPredictedVehicleState(starkShieldApp));

        if (options.promptPredictedStateOnPause && engine.isPredictedStatePromptEnabled()) {
            promptPredictedVehicleState(starkShieldApp);
        }

        CountDownLatch spacePressed = new CountDownLatch(1);
        KeyEventDispatcher dispatcher = event -> {
            if (event.getID() == KeyEvent.KEY_PRESSED && event.getKeyCode() == KeyEvent.VK_SPACE) {
                spacePressed.countDown();
                return true;
            }
            return false;
        };

        KeyboardFocusManager focusManager = KeyboardFocusManager.getCurrentKeyboardFocusManager();
        focusManager.addKeyEventDispatcher(dispatcher);
        try {
            spacePressed.await();
        } finally {
            engine.setPredictedStateQueryAction(null);
            focusManager.removeKeyEventDispatcher(dispatcher);
        }
    }

    private static void promptPredictedVehicleState(StarkShieldApp starkShieldApp) {
        String vehicleId = JOptionPane.showInputDialog(
                null,
                "Vehicle id to print predicted final state (Cancel or empty to skip):",
                "StarkShield prediction",
                JOptionPane.QUESTION_MESSAGE
        );
        if (vehicleId == null || vehicleId.isBlank()) {
            return;
        }
        System.out.println(starkShieldApp.getPredictedFinalVehicleState(vehicleId));
    }

    public static class RunOptions {
        public boolean pauseAfterShieldDecision = false;
        public boolean promptPredictedStateOnPause = false;
        public boolean saveInitialState = true;
        public boolean renderEachStep = true;
        public boolean rethrowOnCrash = true;
        public boolean printDiagnostics = true;
        public int timeForSimulationSeconds = DEFAULT_TIME_FOR_SIMULATION_SECONDS;
        public int shieldPredictFutureSeconds = 3;
        public double shieldEgoRangeMeters = StarkShieldApp.DEFAULT_SHIELD_EGO_RANGE_METERS;
        public boolean randomizeShieldHiddenTargetAndCooldown = StarkShieldApp.DEFAULT_RANDOMIZE_HIDDEN_TARGET_AND_COOLDOWN;
        public boolean readShieldIdmCooldownTimer = StarkShieldApp.DEFAULT_READ_SHIELD_IDM_COOLDOWN_TIMER;
        public boolean checkChangeLaneToRearVehicleThreat = false;
        public String aiProfile = HighwayAiClient.getConfiguredAiProfile();
        public VerifyTimingStats verifyTimingStats = null;
        public boolean useRandomActionGenerator = false;
        public Long randomActionSeed = null;
        public Long probabilityRejectionSeed = null;
        public DecisionMode decisionMode = DecisionMode.STARK_SHIELD;
        public double pureRejectProbability = 0.0;
        public SpeedRejectProbabilityModel speedRejectProbabilityModel = null;
        public String logDir = DEFAULT_LOG_DIR;

        private boolean shouldRejectByProbability(double egoSpeed, Random random) {
            double probability = decisionMode == DecisionMode.SPEED_CONDITIONAL_PROBABILITY
                    && speedRejectProbabilityModel != null
                    ? speedRejectProbabilityModel.probabilityForSpeed(egoSpeed)
                    : pureRejectProbability;
            probability = Math.max(0.0, Math.min(1.0, probability));
            double value = random == null
                    ? java.util.concurrent.ThreadLocalRandom.current().nextDouble()
                    : random.nextDouble();
            return value < probability;
        }
    }

    public enum DecisionMode {
        NO_SHIELD,
        STARK_SHIELD,
        PURE_PROBABILITY,
        SPEED_CONDITIONAL_PROBABILITY
    }

    public static class RunResult {
        public final boolean crashed;

        private RunResult(boolean crashed) {
            this.crashed = crashed;
        }
    }

    public interface SpeedRejectProbabilityModel {
        double probabilityForSpeed(double egoSpeed);
    }

    public static class VerifyTimingStats {
        private static final double SPEED_BUCKET_WIDTH = 5.0;
        private long count = 0;
        private long rejectedCount = 0;
        private double meanNanos = 0.0;
        private double m2Nanos = 0.0;
        private long minNanos = Long.MAX_VALUE;
        private long maxNanos = Long.MIN_VALUE;
        private final java.util.Map<Integer, SpeedRejectBucket> speedRejectBuckets = new java.util.TreeMap<>();

        public synchronized void record(long elapsedNanos) {
            record(elapsedNanos, false, Double.NaN);
        }

        public synchronized void record(long elapsedNanos, boolean rejected, double egoSpeed) {
            count++;
            if (rejected) {
                rejectedCount++;
            }
            double delta = elapsedNanos - meanNanos;
            meanNanos += delta / count;
            double delta2 = elapsedNanos - meanNanos;
            m2Nanos += delta * delta2;
            minNanos = Math.min(minNanos, elapsedNanos);
            maxNanos = Math.max(maxNanos, elapsedNanos);
            if (!Double.isNaN(egoSpeed) && !Double.isInfinite(egoSpeed)) {
                int bucketIndex = (int) Math.floor(egoSpeed / SPEED_BUCKET_WIDTH);
                speedRejectBuckets
                        .computeIfAbsent(bucketIndex, SpeedRejectBucket::new)
                        .record(rejected);
            }
        }

        public synchronized Snapshot snapshot() {
            double varianceNanos = count > 1 ? m2Nanos / (count - 1) : 0.0;
            List<SpeedRejectBucketSnapshot> bucketSnapshots = new ArrayList<>();
            for (SpeedRejectBucket bucket : speedRejectBuckets.values()) {
                bucketSnapshots.add(bucket.snapshot());
            }
            return new Snapshot(count, meanNanos, varianceNanos,
                    count == 0 ? 0 : minNanos,
                    count == 0 ? 0 : maxNanos,
                    rejectedCount,
                    bucketSnapshots);
        }

        public static class Snapshot {
            public final long count;
            public final double meanNanos;
            public final double varianceNanos;
            public final long minNanos;
            public final long maxNanos;
            public final long rejectedCount;
            public final List<SpeedRejectBucketSnapshot> speedRejectBuckets;

            private Snapshot(long count, double meanNanos, double varianceNanos, long minNanos, long maxNanos,
                             long rejectedCount, List<SpeedRejectBucketSnapshot> speedRejectBuckets) {
                this.count = count;
                this.meanNanos = meanNanos;
                this.varianceNanos = varianceNanos;
                this.minNanos = minNanos;
                this.maxNanos = maxNanos;
                this.rejectedCount = rejectedCount;
                this.speedRejectBuckets = speedRejectBuckets;
            }

            public double meanMillis() {
                return meanNanos / 1_000_000.0;
            }

            public double varianceMillisSquared() {
                return varianceNanos / 1_000_000_000_000.0;
            }

            public double stdMillis() {
                return Math.sqrt(varianceNanos) / 1_000_000.0;
            }

            public double minMillis() {
                return minNanos / 1_000_000.0;
            }

            public double maxMillis() {
                return maxNanos / 1_000_000.0;
            }

            public double rejectProbability() {
                return count == 0 ? 0.0 : (double) rejectedCount / count;
            }
        }

        public static class SpeedRejectBucketSnapshot {
            public final double minSpeed;
            public final double maxSpeed;
            public final long count;
            public final long rejectedCount;

            private SpeedRejectBucketSnapshot(double minSpeed, double maxSpeed, long count, long rejectedCount) {
                this.minSpeed = minSpeed;
                this.maxSpeed = maxSpeed;
                this.count = count;
                this.rejectedCount = rejectedCount;
            }

            public double rejectProbability() {
                return count == 0 ? 0.0 : (double) rejectedCount / count;
            }
        }

        private static class SpeedRejectBucket {
            private final int bucketIndex;
            private long count = 0;
            private long rejectedCount = 0;

            private SpeedRejectBucket(int bucketIndex) {
                this.bucketIndex = bucketIndex;
            }

            private void record(boolean rejected) {
                count++;
                if (rejected) {
                    rejectedCount++;
                }
            }

            private SpeedRejectBucketSnapshot snapshot() {
                double minSpeed = bucketIndex * SPEED_BUCKET_WIDTH;
                return new SpeedRejectBucketSnapshot(minSpeed, minSpeed + SPEED_BUCKET_WIDTH, count, rejectedCount);
            }
        }
    }
}
