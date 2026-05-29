package Scenarios;

import Scenarios.Engine.HighwayEngine;

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
        applyLoggedConfiguration(initialStateFile, options);
        options.pauseAfterShieldDecision = PAUSE_AFTER_SHIELD_DECISION;
        options.promptPredictedStateOnPause = PROMPT_PREDICTED_STATE_ON_PAUSE;
        options.saveInitialState = false;
        options.renderEachStep = RENDER_EACH_STEP;
        options.rethrowOnCrash = true;
        options.printDiagnostics = true;

        StarkScenarioRunner.runShieldedScenario(realWorld, options);
    }

    private static void applyLoggedConfiguration(String initialStateFile, StarkScenarioRunner.RunOptions options) {
        Path logPath = Path.of(initialStateFile).getParent();
        if (logPath == null) {
            options.randomizeShieldHiddenTargetAndCooldown = false;
            return;
        }

        Path commentPath = logPath.resolve("comment.md");
        if (!Files.exists(commentPath)) {
            options.randomizeShieldHiddenTargetAndCooldown = false;
            System.out.println("No comment.md found for replay; using passed targetSpeed/cooldown for StarkShield.");
            return;
        }

        try {
            String comment = Files.readString(commentPath);
            options.randomizeShieldHiddenTargetAndCooldown = !comment.contains(
                    "starkShieldTargetSpeedSource: passed from real world vehicle state");
            options.shieldEgoRangeMeters = parseDoubleConfig(comment, "starkShieldRadius", options.shieldEgoRangeMeters);
            options.checkChangeLaneToRearVehicleThreat = parseBooleanConfig(comment,
                    "checkChangeLaneToRearVehicleThreat", options.checkChangeLaneToRearVehicleThreat);
            System.out.printf(
                    "Recovered StarkShield config: randomizeHiddenTargetAndCooldown=%s, radius=%.1f, rearThreatCheck=%s%n",
                    options.randomizeShieldHiddenTargetAndCooldown,
                    options.shieldEgoRangeMeters,
                    options.checkChangeLaneToRearVehicleThreat
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

    private static boolean parseBooleanConfig(String comment, String key, boolean fallback) {
        Matcher matcher = Pattern.compile("-\\s*" + Pattern.quote(key) + ":\\s*(true|false)")
                .matcher(comment);
        return matcher.find() ? Boolean.parseBoolean(matcher.group(1)) : fallback;
    }
}
