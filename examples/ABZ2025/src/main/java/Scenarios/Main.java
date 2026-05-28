package Scenarios;

public class Main {
    public static void main(String[] args) throws Exception {
        //RandomStarkRun.main(args);
       // String filePath = "examples/ABZ2025/src/main/java/Scenarios/logs15/safe_initial_20260528_010757_923_2.json";
//        RecoverStarkRun.main(new String[]{filePath});
//        RecoverProbabilityRun.main(new String[]{filePath});
//        RecoverJointRun.main(new String[]{filePath});

        GenerateInitialLogsRun.GenerationConfig config = new GenerateInitialLogsRun.GenerationConfig();
        config.logDir = "examples/ABZ2025/src/main/java/Scenarios/logs18";
        config.comment = "polite traffic log generation";
        config.populateTargetVehicles = 36;
        config.populateNumLanes = 3;
        config.populateMinX = 0.0;
        config.populateMaxX = 400.0;
        config.polite = false;
        config.shieldEgoRangeMeters = 200.0;
        config.randomizeShieldHiddenTargetAndCooldown = false;
        GenerateInitialLogsRun.run(config);
        CompareWithNoShield.run(config);
        CompareWithPureProbability.run(config);
        CompareWithJointProbability.run(config);
    }
}
