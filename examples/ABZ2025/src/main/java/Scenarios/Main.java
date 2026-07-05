package Scenarios;
import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.HighwayEngine.NpcVehicleType.*;
public class Main {
    public static void main(String[] args) throws Exception {
       // RandomStarkRun.main(args);
        //String filePath = "examples/ABZ2025/src/main/java/Scenarios/logs70/crash_initial_20260706_002556_862_90.json";
       //RecoverStarkRun.main(new String[]{filePath});
//        RecoverProbabilityRun.main(new String[]{filePath});
//        RecoverJointRun.main(new String[]{filePath});
        boolean gen = true;
        gen = false;
        gen = true;
        if(gen) {
            GenerateInitialLogsRun.GenerationConfig config = new GenerateInitialLogsRun.GenerationConfig();
            config.logDir = "examples/ABZ2025/src/main/java/Scenarios/logs74";
            config.comment = "polite traffic log generation";
            config.aiProfile = "base"; // or "adversarial"
            config.aiProfile = "adversarial";
            config.populateTargetVehicles = 20;
            config.populateNumLanes = 3;
            config.populateMinX = 0.0;
            config.populateMaxX = 400.0;
            config.placeEgoAtTrafficMiddle = false; // ego x=(minX+maxX)/2, here x=200m
            config.egoCentered = false; // ego starts in the middle lane
            config.polite = false;

            config.npcVehicleType = HighwayEngine.NpcVehicleType.DEFAULT;
            config.npcIdmActionStepLength = 0;
            config.npcReactionDelay = 0.3;
            config.idmTimeWanted = 1.0;

            config.shieldEgoRangeMeters = 200.0;
            config.randomizeShieldHiddenTargetAndCooldown = true;
            config.checkChangeLaneToRearVehicleThreat = true;
            config.readShieldIdmCooldownTimer = false;
            config.continueAfterNpcCollision = true;

            config.decisionMode = StarkScenarioRunner.DecisionMode.STARK_SHIELD;
            config.useInstantProtectedCar = false;
            config.instantAiDecisionIntervalSeconds = 0.25;
            config.instantShieldPredictionSeconds = 0.4;
            config.instantShieldAiActionSeconds = 0.1;



            //config.enableOvertakeGate = true;
            config.aggressiveFinalStability = false;
            //this is for adjusting targetSpeed
            config.fixPrediction = true;
            GenerateInitialLogsRun.run(config);
            CompareWithNoShield.run(config);
            CompareWithPureProbability.run(config);
            CompareWithJointProbability.run(config);
        }
    }
}
