package Scenarios;

import Scenarios.Engine.HighwayEngine;

import java.nio.file.Path;

public class RecoverRandomActionRun {
    private static  String INITIAL_STATE_FILE =
            "examples/ABZ2025/src/main/java/Scenarios/random_action_compare/debug_crashes/random_action_stark_shield_with_rear_threat_check/";
    private static final boolean USE_STARK_SHIELD = true;
    private static final boolean CHECK_CHANGE_LANE_TO_REAR_VEHICLE_THREAT = true;
    private static final boolean RANDOMIZE_SHIELD_HIDDEN_TARGET_AND_COOLDOWN = false;
    private static final boolean PAUSE_AFTER_SHIELD_DECISION = true;
    private static final boolean RENDER_EACH_STEP = true;

    public static void main(String[] args) throws Exception {
        INITIAL_STATE_FILE += "CRASH_WITH_REAR_THREAT_random_initial_0054_seed_202605280054.json";
        String stateFile = args != null && args.length > 0 ? args[0] : INITIAL_STATE_FILE;
        long seed = args != null && args.length > 1
                ? Long.parseLong(args[1])
                : RandomActionCompareTest.stableSeed(Path.of(stateFile));

        HighwayEngine realWorld = StarkScenarioRunner.createWorldFromLog(
                stateFile,
                StarkScenarioRunner.DEFAULT_DT
        );

        StarkScenarioRunner.RunOptions options = new StarkScenarioRunner.RunOptions();
        options.decisionMode = USE_STARK_SHIELD
                ? StarkScenarioRunner.DecisionMode.STARK_SHIELD
                : StarkScenarioRunner.DecisionMode.NO_SHIELD;
        options.useRandomActionGenerator = true;
        options.randomActionSeed = seed;
        options.checkChangeLaneToRearVehicleThreat = CHECK_CHANGE_LANE_TO_REAR_VEHICLE_THREAT;
        options.randomizeShieldHiddenTargetAndCooldown = RANDOMIZE_SHIELD_HIDDEN_TARGET_AND_COOLDOWN;
        options.pauseAfterShieldDecision = PAUSE_AFTER_SHIELD_DECISION;
        options.saveInitialState = false;
        options.renderEachStep = RENDER_EACH_STEP;
        options.rethrowOnCrash = true;
        options.printDiagnostics = true;

        System.out.printf("Recover random action run: state=%s seed=%d shield=%s rearThreatCheck=%s randomizeHiddenTargetCooldown=%s%n",
                stateFile, seed, USE_STARK_SHIELD, CHECK_CHANGE_LANE_TO_REAR_VEHICLE_THREAT,
                RANDOMIZE_SHIELD_HIDDEN_TARGET_AND_COOLDOWN);
        StarkScenarioRunner.runScenario(realWorld, options);
    }
}
