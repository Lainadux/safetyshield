package Scenarios;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class GenerateSafeControllerLogsRun {
    private static final DateTimeFormatter RUN_DIR_TIME_FORMAT =
            DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");
    private static final int LOG_COUNT = 100;
    private static final int THREAD_COUNT = 4;
    private static final String LOG_DIR = "examples/ABZ2025/src/main/java/Scenarios/logs_safe_controller";
    private static final int SHIELD_PREDICT_FUTURE_SECONDS = 2;
    private static final double SHIELD_EGO_RANGE_METERS = 100.0;
    private static final double MAX_DESIRED_SPEED = 30.0;
    private static final double AGGRESSIVE_V3_TTC_THRESHOLD = 3.5;
    private static final double NPC_INITIAL_SPEED_MIN = 21.0;
    private static final double NPC_INITIAL_SPEED_MAX = 24.0;
    private static final StarkScenarioRunner.FinalStabilityMode FINAL_STABILITY_MODE =
            StarkScenarioRunner.FinalStabilityMode.V2;

    public static void main(String[] args) throws Exception {
        GenerateInitialLogsRun.GenerationConfig config = new GenerateInitialLogsRun.GenerationConfig();
        config.logCount = LOG_COUNT;
        config.threadCount = THREAD_COUNT;
        config.logDir = LOG_DIR + "/run_" + LocalDateTime.now().format(RUN_DIR_TIME_FORMAT);
        config.maxDesiredSpeed = MAX_DESIRED_SPEED;
        config.shieldPredictFutureSeconds = SHIELD_PREDICT_FUTURE_SECONDS;
        config.shieldEgoRangeMeters = SHIELD_EGO_RANGE_METERS;
        config.finalStabilityMode = FINAL_STABILITY_MODE;
        config.aggressiveV3TtcThreshold = AGGRESSIVE_V3_TTC_THRESHOLD;
        config.npcInitialSpeedMin = NPC_INITIAL_SPEED_MIN;
        config.npcInitialSpeedMax = NPC_INITIAL_SPEED_MAX;

        config.comment = "safe controller log generation";
        config.decisionMode = StarkScenarioRunner.DecisionMode.SAFE_CONTROLLER;
        config.checkChangeLaneToRearVehicleThreat = true;
        config.randomizeShieldHiddenTargetAndCooldown = true;
        config.readShieldIdmCooldownTimer = false;
        config.continueAfterNpcCollision = true;

        GenerateInitialLogsRun.run(config);
    }
}
