package Scenarios;

import Scenarios.Engine.HighwayEngine;

import java.nio.file.Path;

public class RecoverProbabilityRun {
    private static final String INITIAL_STATE_FILE =
            "examples/ABZ2025/src/main/java/Scenarios/logs14/crash_initial_20260527_235021_384_1.json";
    private static final boolean PAUSE_AFTER_SHIELD_DECISION = true;
    private static final boolean RENDER_EACH_STEP = true;

    public static void main(String[] args) throws Exception {
        String stateFile = args != null && args.length > 0 ? args[0] : INITIAL_STATE_FILE;
        Path statePath = Path.of(stateFile);
        Path logDir = statePath.getParent();
        CompareExperimentSupport.TimingProfile timingProfile =
                CompareExperimentSupport.readTimingProfile(logDir);

        HighwayEngine realWorld = StarkScenarioRunner.createWorldFromLog(
                stateFile,
                StarkScenarioRunner.DEFAULT_DT
        );

        StarkScenarioRunner.RunOptions options = new StarkScenarioRunner.RunOptions();
        options.decisionMode = StarkScenarioRunner.DecisionMode.PURE_PROBABILITY;
        options.pureRejectProbability = timingProfile.rejectProbability();
        options.pauseAfterShieldDecision = PAUSE_AFTER_SHIELD_DECISION;
        options.saveInitialState = false;
        options.renderEachStep = RENDER_EACH_STEP;
        options.rethrowOnCrash = true;
        options.printDiagnostics = true;

        StarkScenarioRunner.runScenario(realWorld, options);
    }
}
