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
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicInteger;

public final class StarkScenarioRunner {
    public static final double DEFAULT_DT = 0.02;
    public static final int DEFAULT_TIME_FOR_SIMULATION_SECONDS = 40;
    public static final String DEFAULT_LOG_DIR = "examples/ABZ2025/src/main/java/Scenarios/logs";

    private static final DateTimeFormatter STATE_TIME_FORMAT = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS");
    private static final AtomicInteger SAVED_STATE_SEQUENCE = new AtomicInteger(0);

    private StarkScenarioRunner() {
    }

    public static HighwayEngine createRandomWorld(double dt) {
        HighwayEngine realWorld = new HighwayEngine(dt, true);
        realWorld.populateTraffic(18, 3, 0.0, 200);
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
        double dt = realWorld.dt;
        ProtectedControlledVehicle protectedControlledVehicle = ensureProtectedEgo(realWorld);
        List<Vehicle> initialScenario = deepCopyVehicles(realWorld.vehicles);

        boolean crashed = false;
        boolean initialScenarioSaved = false;
        double prevTgtspd = 0.0;
        int prevCurrentLane = 0;
        long lastStep = (long) options.timeForSimulationSeconds * realWorld.STEPS_PER_SECOND - 1;

        try {
            while (realWorld.stepCount != lastStep) {
                if (realWorld.stepCount % realWorld.STEPS_PER_SECOND == 0) {
                    prevTgtspd = protectedControlledVehicle.targetSpeed;
                    prevCurrentLane = protectedControlledVehicle.getLaneIndex();
                    protectedControlledVehicle.fetchDesiredLaneAndTargetSpeed();
                }

                if (realWorld.stepCount % realWorld.STEPS_PER_SECOND == 0) {
                    List<Vehicle> shieldVehicles = buildShieldVehicles(realWorld.vehicles);
                    HighwayEngine shieldEngine = new HighwayEngine(dt, true, true, shieldVehicles);
                    StarkShieldApp starkShieldApp = shieldEngine.createStarkShieldApp(3);
                    boolean isSafe = starkShieldApp.verifySafe();
                    HighwayAiClient.AiDecision decision = protectedControlledVehicle.getLastAiDecision();

                    if (options.printDiagnostics) {
                        System.out.printf("%s AI decision: action=%d, action_name=%s%n",
                                isSafe ? "Safe" : "Unsafe", decision.action, decision.action_name);
                        System.out.println(starkShieldApp.getUnsafeDiagnosis());
                    }

                    if (!isSafe) {
                        protectedControlledVehicle.targetSpeed = prevTgtspd - 5 < 0 ? 0 : prevTgtspd - 5;
                        protectedControlledVehicle.setTargetLaneIndex(prevCurrentLane);
                    }

                    if (options.pauseAfterShieldDecision) {
                        waitForSpaceToContinue(realWorld);
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

    private static void waitForSpaceToContinue(HighwayEngine engine) throws InterruptedException {
        engine.render();

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
            focusManager.removeKeyEventDispatcher(dispatcher);
        }
    }

    public static class RunOptions {
        public boolean pauseAfterShieldDecision = false;
        public boolean saveInitialState = true;
        public boolean renderEachStep = true;
        public boolean rethrowOnCrash = true;
        public boolean printDiagnostics = true;
        public int timeForSimulationSeconds = DEFAULT_TIME_FOR_SIMULATION_SECONDS;
        public String logDir = DEFAULT_LOG_DIR;
    }
}
