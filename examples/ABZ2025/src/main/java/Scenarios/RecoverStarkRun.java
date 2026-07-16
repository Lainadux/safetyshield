package Scenarios;

import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.StarkShieldApp;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class RecoverStarkRun {
    private static final String INITIAL_STATE_FILE =
            "examples/ABZ2025/src/main/java/Scenarios/logs/crash_initial_20260526_163253_672.json";
    private static final boolean PAUSE_AFTER_SHIELD_DECISION = true;
    private static final boolean RENDER_EACH_STEP = true;
    private static final boolean PROMPT_PREDICTED_STATE_ON_PAUSE = true;

    public static void main(String[] args) throws Exception {
        HighwayEngine realWorld;
        String initialStateFile;
        if(args != null && args.length > 0){
            initialStateFile = args[0];
            realWorld = StarkScenarioRunner.createWorldFromLog(
                    initialStateFile,
                    StarkScenarioRunner.DEFAULT_DT
            );
        }else{
            initialStateFile = INITIAL_STATE_FILE;
            realWorld = StarkScenarioRunner.createWorldFromLog(
                    initialStateFile,
                    StarkScenarioRunner.DEFAULT_DT
            );
        }

        StarkScenarioRunner.RunOptions options = new StarkScenarioRunner.RunOptions();
        applyLoggedConfiguration(initialStateFile, realWorld, options);
        options.aggressiveV2MaxStableRelativeSpeed = parseForcedAggressiveV2Threshold(args);
        options.pauseAfterShieldDecision = PAUSE_AFTER_SHIELD_DECISION;
        options.promptPredictedStateOnPause = PROMPT_PREDICTED_STATE_ON_PAUSE;
        options.saveInitialState = false;
        options.renderEachStep = RENDER_EACH_STEP;
        options.rethrowOnCrash = true;
        options.printDiagnostics = true;

        StarkScenarioRunner.runShieldedScenario(realWorld, options);
    }

    private static double parseForcedAggressiveV2Threshold(String[] args) {
        if (args == null || args.length < 2 || args[1] == null || args[1].isBlank()) {
            return StarkShieldApp.DEFAULT_AGGRESSIVE_V2_MAX_STABLE_RELATIVE_SPEED;
        }
        try {
            return Double.parseDouble(args[1]);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException(
                    "Second argument must be aggressiveV2MaxStableRelativeSpeed, for example: RecoverStarkRun.main(new String[]{filePath, \"4\"})",
                    e
            );
        }
    }

    private static void applyLoggedConfiguration(String initialStateFile, HighwayEngine realWorld,
                                                 StarkScenarioRunner.RunOptions options) {
        Path initialStatePath = Path.of(initialStateFile);
        Path metadataPath = Path.of(initialStateFile + ".meta.md");
        Path logPath = initialStatePath.getParent();
        if (logPath == null) {
            options.randomizeShieldHiddenTargetAndCooldown = false;
            return;
        }

        Path commentPath = logPath.resolve("comment.md");
        if (!Files.exists(commentPath) && !Files.exists(metadataPath)) {
            options.randomizeShieldHiddenTargetAndCooldown = false;
            System.out.println("No comment.md or replay metadata found; using passed targetSpeed/cooldown for StarkShield.");
            return;
        }

        try {
            String comment = "";
            if (Files.exists(metadataPath)) {
                comment += Files.readString(metadataPath) + System.lineSeparator();
            }
            if (Files.exists(commentPath)) {
                comment += Files.readString(commentPath);
            }
            options.randomizeShieldHiddenTargetAndCooldown = !comment.contains(
                    "starkShieldTargetSpeedSource: passed from real world vehicle state");
            options.randomizeShieldHiddenTargetAndCooldown = parseBooleanConfig(comment,
                    "randomizeShieldHiddenTargetAndCooldown", options.randomizeShieldHiddenTargetAndCooldown);
            options.shieldHiddenStateRandomSeed = parseLongConfig(comment,
                    "shieldHiddenStateRandomSeed", options.shieldHiddenStateRandomSeed);
            options.shieldPredictFutureSeconds = parseIntConfig(comment,
                    "shieldPredictFutureSeconds", options.shieldPredictFutureSeconds);
            options.maxDesiredSpeed = parseDoubleConfig(comment,
                    "maxDesiredSpeed", options.maxDesiredSpeed);
            if (options.randomizeShieldHiddenTargetAndCooldown && options.shieldHiddenStateRandomSeed == null) {
                System.out.println("Warning: StarkShield hidden target/cooldown is randomized but no shieldHiddenStateRandomSeed was logged; replay will not be deterministic.");
            }
            options.readShieldIdmCooldownTimer = !comment.contains(
                    "starkShieldIdmCooldownTimerSource: randomized");
            options.shieldEgoRangeMeters = parseEmbeddedDoubleConfig(comment, "starkShieldRadius", options.shieldEgoRangeMeters);
            options.checkChangeLaneToRearVehicleThreat = parseBooleanConfig(comment,
                    "checkChangeLaneToRearVehicleThreat", options.checkChangeLaneToRearVehicleThreat);
            options.fixPrediction = parseBooleanConfig(comment,
                    "fixPrediction", options.fixPrediction);
            options.finalStabilityMode = parseFinalStabilityModeConfig(comment,
                    "finalStabilityMode", options.finalStabilityMode);
            options.aggressiveFinalStability = parseBooleanConfig(comment,
                    "aggressiveFinalStability", options.aggressiveFinalStability);
            options.finalStabilityPenaltyMode = parseFinalStabilityPenaltyModeConfig(comment,
                    "finalStabilityPenaltyMode", options.finalStabilityPenaltyMode);
            options.aggressiveV2MaxStableRelativeSpeed = parseDoubleConfig(comment,
                    "aggressiveV2MaxStableRelativeSpeed", options.aggressiveV2MaxStableRelativeSpeed);
            options.aggressiveV3TtcThreshold = parseDoubleConfig(comment,
                    "aggressiveV3TtcThreshold", options.aggressiveV3TtcThreshold);
            options.enableOvertakeGate = parseBooleanConfig(comment,
                    "enableOvertakeGate", options.enableOvertakeGate);
            options.continueAfterNpcCollision = parseBooleanConfig(comment,
                    "continueAfterNpcCollision", options.continueAfterNpcCollision);
            options.decisionMode = parseDecisionModeConfig(comment, "decisionMode", options.decisionMode);
            options.useInstantProtectedCar = parseBooleanConfig(comment,
                    "useInstantProtectedCar", options.useInstantProtectedCar);
            options.instantAiDecisionIntervalSeconds = parseDoubleConfig(comment,
                    "instantAiDecisionIntervalSeconds", options.instantAiDecisionIntervalSeconds);
            options.instantShieldPredictionSeconds = parseDoubleConfig(comment,
                    "instantShieldPredictionSeconds", options.instantShieldPredictionSeconds);
            options.instantShieldAiActionSeconds = parseDoubleConfig(comment,
                    "instantShieldAiActionSeconds", options.instantShieldAiActionSeconds);
            options.aiProfile = parseStringConfig(comment, "aiProfile", options.aiProfile);
            String npcVehicleType = parseStringConfig(comment, "realWorldNpcVehicleType", "DEFAULT");
            if (!"DEFAULT".equals(npcVehicleType)) {
                realWorld.idmTimeWanted = parseDoubleConfig(comment, "realWorldIdmTimeWanted", realWorld.idmTimeWanted);
            }
            System.out.printf(
                    "Recovered StarkShield config: aiProfile=%s, decisionMode=%s, instantProtectedCar=%s, npcVehicleType=%s, idmTimeWanted=%.3f, predictionSeconds=%d, maxDesiredSpeed=%.3f, randomizeHiddenTargetAndCooldown=%s, shieldHiddenStateRandomSeed=%s, readIdmCooldownTimer=%s, fixPrediction=%s, finalStabilityMode=%s, aggressiveV2MaxStableRelativeSpeed=%.3f, aggressiveV3TtcThreshold=%.3f, enableOvertakeGate=%s, radius=%.1f, rearThreatCheck=%s, continueAfterNpcCollision=%s%n",
                    options.aiProfile,
                    options.decisionMode,
                    options.useInstantProtectedCar,
                    npcVehicleType,
                    realWorld.idmTimeWanted,
                    options.shieldPredictFutureSeconds,
                    options.maxDesiredSpeed,
                    options.randomizeShieldHiddenTargetAndCooldown,
                    options.shieldHiddenStateRandomSeed,
                    options.readShieldIdmCooldownTimer,
                    options.fixPrediction,
                    options.resolvedFinalStabilityMode(),
                    options.aggressiveV2MaxStableRelativeSpeed,
                    options.aggressiveV3TtcThreshold,
                    options.enableOvertakeGate,
                    options.shieldEgoRangeMeters,
                    options.checkChangeLaneToRearVehicleThreat,
                    options.continueAfterNpcCollision
            );
        } catch (IOException e) {
            options.randomizeShieldHiddenTargetAndCooldown = false;
            System.out.println("Could not read comment.md for replay; using passed targetSpeed/cooldown for StarkShield.");
        }
    }

    private static double parseDoubleConfig(String comment, String key, double fallback) {
        Matcher matcher = Pattern.compile("-\\s*" + Pattern.quote(key) + ":\\s*([-+]?\\d+(?:\\.\\d+)?)")
                .matcher(comment);
        return matcher.find() ? Double.parseDouble(matcher.group(1)) : fallback;
    }

    private static double parseEmbeddedDoubleConfig(String comment, String key, double fallback) {
        Matcher matcher = Pattern.compile("-\\s*" + Pattern.quote(key) + ":\\s*[^\\r\\n]*?([-+]?\\d+(?:\\.\\d+)?)")
                .matcher(comment);
        return matcher.find() ? Double.parseDouble(matcher.group(1)) : fallback;
    }

    private static boolean parseBooleanConfig(String comment, String key, boolean fallback) {
        Matcher matcher = Pattern.compile("-\\s*" + Pattern.quote(key) + ":\\s*(true|false)")
                .matcher(comment);
        return matcher.find() ? Boolean.parseBoolean(matcher.group(1)) : fallback;
    }

    private static Long parseLongConfig(String comment, String key, Long fallback) {
        Matcher matcher = Pattern.compile("-\\s*" + Pattern.quote(key) + ":\\s*(-?\\d+)")
                .matcher(comment);
        return matcher.find() ? Long.parseLong(matcher.group(1)) : fallback;
    }

    private static int parseIntConfig(String comment, String key, int fallback) {
        Matcher matcher = Pattern.compile("-\\s*" + Pattern.quote(key) + ":\\s*(-?\\d+)")
                .matcher(comment);
        return matcher.find() ? Integer.parseInt(matcher.group(1)) : fallback;
    }

    private static String parseStringConfig(String comment, String key, String fallback) {
        Matcher matcher = Pattern.compile("-\\s*" + Pattern.quote(key) + ":\\s*`?([^`\\r\\n]+)`?")
                .matcher(comment);
        return matcher.find() ? matcher.group(1).trim() : fallback;
    }

    private static StarkScenarioRunner.DecisionMode parseDecisionModeConfig(
            String comment, String key, StarkScenarioRunner.DecisionMode fallback) {
        String value = parseStringConfig(comment, key, fallback.name());
        try {
            return StarkScenarioRunner.DecisionMode.valueOf(value);
        } catch (IllegalArgumentException e) {
            return fallback;
        }
    }

    private static StarkShieldApp.FinalStabilityPenaltyMode parseFinalStabilityPenaltyModeConfig(
            String comment, String key, StarkShieldApp.FinalStabilityPenaltyMode fallback) {
        String value = parseStringConfig(comment, key, fallback.name());
        try {
            return StarkShieldApp.FinalStabilityPenaltyMode.valueOf(value);
        } catch (IllegalArgumentException e) {
            return fallback;
        }
    }

    private static StarkScenarioRunner.FinalStabilityMode parseFinalStabilityModeConfig(
            String comment, String key, StarkScenarioRunner.FinalStabilityMode fallback) {
        String value = parseStringConfig(comment, key, fallback == null ? "" : fallback.name());
        if (value == null || value.isBlank()) {
            return fallback;
        }
        try {
            return StarkScenarioRunner.FinalStabilityMode.valueOf(value);
        } catch (IllegalArgumentException e) {
            return fallback;
        }
    }
}
