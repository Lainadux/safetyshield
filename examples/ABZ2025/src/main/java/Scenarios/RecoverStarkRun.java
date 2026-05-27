package Scenarios;

import Scenarios.Engine.HighwayEngine;

public class RecoverStarkRun {
    private static final String INITIAL_STATE_FILE =
            "examples/ABZ2025/src/main/java/Scenarios/logs/crash_initial_20260526_163253_672.json";
    private static final boolean PAUSE_AFTER_SHIELD_DECISION = true;
    private static final boolean RENDER_EACH_STEP = true;
    private static final boolean PROMPT_PREDICTED_STATE_ON_PAUSE = true;

    public static void main(String[] args) throws Exception {
        HighwayEngine realWorld;
        if(args != null && args.length > 0){
            realWorld = StarkScenarioRunner.createWorldFromLog(
                    args[0],
                    StarkScenarioRunner.DEFAULT_DT
            );
        }else{
            realWorld = StarkScenarioRunner.createWorldFromLog(
                    INITIAL_STATE_FILE,
                    StarkScenarioRunner.DEFAULT_DT
            );
        }

        StarkScenarioRunner.RunOptions options = new StarkScenarioRunner.RunOptions();
        options.pauseAfterShieldDecision = PAUSE_AFTER_SHIELD_DECISION;
        options.promptPredictedStateOnPause = PROMPT_PREDICTED_STATE_ON_PAUSE;
        options.saveInitialState = false;
        options.renderEachStep = RENDER_EACH_STEP;
        options.rethrowOnCrash = true;
        options.printDiagnostics = true;

        StarkScenarioRunner.runShieldedScenario(realWorld, options);
    }
}
