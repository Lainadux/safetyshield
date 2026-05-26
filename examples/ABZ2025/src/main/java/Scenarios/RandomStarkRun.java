package Scenarios;

import Scenarios.Engine.HighwayEngine;

public class RandomStarkRun {
    private static final boolean PAUSE_AFTER_SHIELD_DECISION = true;
    private static final boolean SAVE_INITIAL_STATE = true;
    private static final boolean RENDER_EACH_STEP = true;
    private static final String LOG_DIR = StarkScenarioRunner.DEFAULT_LOG_DIR;

    public static void main(String[] args) throws Exception {
        HighwayEngine realWorld = StarkScenarioRunner.createRandomWorld(StarkScenarioRunner.DEFAULT_DT);

        StarkScenarioRunner.RunOptions options = new StarkScenarioRunner.RunOptions();
        options.pauseAfterShieldDecision = PAUSE_AFTER_SHIELD_DECISION;
        options.saveInitialState = SAVE_INITIAL_STATE;
        options.renderEachStep = RENDER_EACH_STEP;
        options.rethrowOnCrash = true;
        options.printDiagnostics = true;
        options.logDir = LOG_DIR;

        StarkScenarioRunner.runShieldedScenario(realWorld, options);
    }
}
