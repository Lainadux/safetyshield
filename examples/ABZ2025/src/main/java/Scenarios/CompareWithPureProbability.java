package Scenarios;

import java.nio.file.Path;

public class CompareWithPureProbability {
    public static void run(GenerateInitialLogsRun.GenerationConfig config) throws Exception {
        Path logDir = Path.of(config.logDir);
        CompareExperimentSupport.TimingProfile timingProfile =
                CompareExperimentSupport.readTimingProfile(logDir);
        CompareExperimentSupport.runProbabilityExperiment(
                config,
                "pure_probability",
                StarkScenarioRunner.DecisionMode.PURE_PROBABILITY,
                timingProfile
        );
    }
}
