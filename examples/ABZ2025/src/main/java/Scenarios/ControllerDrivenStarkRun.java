package Scenarios;

import Scenarios.Engine.HighwayEngine;

public class ControllerDrivenStarkRun {
    private static final boolean PAUSE_AFTER_SHIELD_DECISION = true;
    private static final boolean SAVE_INITIAL_STATE = true;
    private static final boolean RENDER_EACH_STEP = true;
    private static final boolean PROMPT_PREDICTED_STATE_ON_PAUSE = true;
    private static final String LOG_DIR = StarkScenarioRunner.DEFAULT_LOG_DIR;

    public static void main(String[] args) throws Exception {
        HighwayEngine realWorld = StarkScenarioRunner.createRandomWorld(StarkScenarioRunner.DEFAULT_DT);

        StarkScenarioRunner.RunOptions options = new StarkScenarioRunner.RunOptions();
        options.decisionMode = StarkScenarioRunner.DecisionMode.CONTROLLER_DRIVEN_STARK_SHIELD;
        options.pauseAfterShieldDecision = PAUSE_AFTER_SHIELD_DECISION;
        options.promptPredictedStateOnPause = PROMPT_PREDICTED_STATE_ON_PAUSE;
        options.saveInitialState = SAVE_INITIAL_STATE;
        options.renderEachStep = RENDER_EACH_STEP;
        options.rethrowOnCrash = true;
        options.printDiagnostics = true;
        options.logDir = LOG_DIR;

        options.aiProfile = args.length > 0 ? args[0] : "adversarial";
        options.shieldPredictFutureSeconds = args.length > 1 ? Integer.parseInt(args[1]) : 3;
        options.shieldEgoRangeMeters = args.length > 2 ? Double.parseDouble(args[2]) : 100.0;
        options.finalStabilityMode = args.length > 3
                ? parseFinalStabilityMode(args[3])
                : StarkScenarioRunner.FinalStabilityMode.V2;
        options.checkChangeLaneToRearVehicleThreat = true;
        options.randomizeShieldHiddenTargetAndCooldown = true;

        StarkScenarioRunner.runShieldedScenario(realWorld, options);
    }

    private static StarkScenarioRunner.FinalStabilityMode parseFinalStabilityMode(String value) {
        String normalized = value == null ? "" : value.trim().toUpperCase();
        if ("V4".equals(normalized)) {
            normalized = "V4_RSS";
        }
        return StarkScenarioRunner.FinalStabilityMode.valueOf(normalized);
    }
}
