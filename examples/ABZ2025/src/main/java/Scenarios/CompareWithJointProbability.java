package Scenarios;

import java.nio.file.Path;

public class CompareWithJointProbability {
    public static void run(GenerateInitialLogsRun.GenerationConfig config) throws Exception {
        Path logDir = Path.of(config.logDir);
        CompareExperimentSupport.TimingProfile timingProfile =
                CompareExperimentSupport.readTimingProfile(logDir);
        CompareExperimentSupport.runProbabilityExperiment(
                config,
                "joint_speed_probability",
                StarkScenarioRunner.DecisionMode.SPEED_CONDITIONAL_PROBABILITY,
                timingProfile
        );
    }
}
