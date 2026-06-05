package Scenarios;
import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.HighwayEngine.NpcVehicleType.*;
public class Main {
    public static void main(String[] args) throws Exception {
        //RandomStarkRun.main(args);
//        String filePath = "examples/ABZ2025/src/main/java/Scenarios/logs30/crash_initial_20260605_165845_304_17.json";
//        RecoverStarkRun.main(new String[]{filePath});
//        RecoverProbabilityRun.main(new String[]{filePath});
//        RecoverJointRun.main(new String[]{filePath});
        boolean gen = true;
        //gen = false;
        //gen = true;
        if(gen) {
            GenerateInitialLogsRun.GenerationConfig config = new GenerateInitialLogsRun.GenerationConfig();
            config.logDir = "examples/ABZ2025/src/main/java/Scenarios/logs31";
            config.comment = "polite traffic log generation";
            config.aiProfile = "base"; // or "adversarial"
            config.aiProfile = "adversarial";
            config.populateTargetVehicles = 36;
            config.populateNumLanes = 3;
            config.populateMinX = 0.0;
            config.populateMaxX = 400.0;
            config.placeEgoAtTrafficMiddle = true; // ego x=(minX+maxX)/2, here x=200m
            config.egoCentered = true; // ego starts in the middle lane
            config.polite = false;
            config.npcVehicleType = HighwayEngine.NpcVehicleType.IDM_COOLDOWN;
            config.npcIdmActionStepLength = 0.1;
            config.shieldEgoRangeMeters = 200.0;
            config.randomizeShieldHiddenTargetAndCooldown = false;
            config.checkChangeLaneToRearVehicleThreat = true;
            config.readShieldIdmCooldownTimer = false;
            GenerateInitialLogsRun.run(config);
//            CompareWithNoShield.run(config);
//            CompareWithPureProbability.run(config);
//            CompareWithJointProbability.run(config);
        }
    }
}
