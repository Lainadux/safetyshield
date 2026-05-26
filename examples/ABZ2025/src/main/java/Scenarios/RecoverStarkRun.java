package Scenarios;

import Scenarios.Engine.HighwayEngine;

public class RecoverStarkRun {
    private static final String INITIAL_STATE_FILE =
            "examples/ABZ2025/src/main/java/Scenarios/logs/crash_initial_20260526_163253_672.json";
    private static final boolean PAUSE_AFTER_SHIELD_DECISION = false;
    private static final boolean RENDER_EACH_STEP = true;

    public static void main(String args) throws Exception {
        HighwayEngine realWorld;
        if(args!=null){
            realWorld = StarkScenarioRunner.createWorldFromLog(
                    args,
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
        options.saveInitialState = false;
        options.renderEachStep = RENDER_EACH_STEP;
        options.rethrowOnCrash = true;
        options.printDiagnostics = true;

        StarkScenarioRunner.runShieldedScenario(realWorld, options);
    }
}
