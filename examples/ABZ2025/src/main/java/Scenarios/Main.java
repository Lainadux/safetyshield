package Scenarios;
import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.HighwayEngine.NpcVehicleType.*;
public class Main {
    public static void main(String[] args) throws Exception {
       // RandomStarkRun.main(args);
        String filePath = "examples/ABZ2025/src/main/java/Scenarios/logs43/crash_initial_20260607_222429_786_6.json";
        RecoverStarkRun.main(new String[]{filePath});
//        RecoverProbabilityRun.main(new String[]{filePath});
//        RecoverJointRun.main(new String[]{filePath});
        boolean gen = true;
        gen = false;
        //gen = true;
        if(gen) {
            GenerateInitialLogsRun.GenerationConfig config = new GenerateInitialLogsRun.GenerationConfig();
            config.logDir = "examples/ABZ2025/src/main/java/Scenarios/logs44";
            config.comment = "polite traffic log generation";
            config.aiProfile = "base"; // or "adversarial"
            config.aiProfile = "adversarial";
            config.populateTargetVehicles = 27;
            config.populateNumLanes = 3;
            config.populateMinX = 0.0;
            config.populateMaxX = 600.0;
            config.placeEgoAtTrafficMiddle = true; // ego x=(minX+maxX)/2, here x=200m
            config.egoCentered = true; // ego starts in the middle lane
            config.polite = false;

            config.npcVehicleType = HighwayEngine.NpcVehicleType.DELAYED_IDM;
            config.npcIdmActionStepLength = 0;
            config.npcReactionDelay = 0.3;
            config.idmTimeWanted = 1.0;
            config.shieldEgoRangeMeters = 200.0;
            config.randomizeShieldHiddenTargetAndCooldown = false;
            config.checkChangeLaneToRearVehicleThreat = true;
            config.readShieldIdmCooldownTimer = false;

            config.decisionMode = StarkScenarioRunner.DecisionMode.INSTANT_BASED_STARK_SHIELD;
            config.useInstantProtectedCar = true;
            config.instantAiDecisionIntervalSeconds = 0.25;
            config.instantShieldPredictionSeconds = 0.4;
            config.instantShieldAiActionSeconds = 0.1;
            GenerateInitialLogsRun.run(config);
//            CompareWithNoShield.run(config);
//            CompareWithPureProbability.run(config);
//            CompareWithJointProbability.run(config);
        }
    }
}
