package Scenarios;

public class CompareWithNoShield {
    public static void run(GenerateInitialLogsRun.GenerationConfig config) throws Exception {
        CompareExperimentSupport.runExperiment(
                config,
                "no_shield",
                StarkScenarioRunner.DecisionMode.NO_SHIELD,
                null
        );
    }
}
