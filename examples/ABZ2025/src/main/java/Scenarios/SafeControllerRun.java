package Scenarios;

import Scenarios.Engine.HighwayEngine;

public class SafeControllerRun {
    private static final boolean PAUSE_AFTER_SHIELD_DECISION = true;
    private static final boolean SAVE_INITIAL_STATE = true;
    private static final boolean RENDER_EACH_STEP = true;
    private static final boolean PROMPT_PREDICTED_STATE_ON_PAUSE = true;
    private static final String LOG_DIR = StarkScenarioRunner.DEFAULT_LOG_DIR;
    private static final double NPC_INITIAL_SPEED_MIN = 20.0;
    private static final double NPC_INITIAL_SPEED_MAX = 24.0;

    public static void main(String[] args) throws Exception {
        HighwayEngine realWorld = new HighwayEngine(StarkScenarioRunner.DEFAULT_DT, true);
        realWorld.npcInitialSpeedMin = NPC_INITIAL_SPEED_MIN;
        realWorld.npcInitialSpeedMax = NPC_INITIAL_SPEED_MAX;
        realWorld.populateTraffic(36, 3, 0.0, 400);
        realWorld.enhancedCollisionCheckEnabled = true;

        StarkScenarioRunner.RunOptions options = new StarkScenarioRunner.RunOptions();
        options.decisionMode = StarkScenarioRunner.DecisionMode.SAFE_CONTROLLER;
        options.pauseAfterShieldDecision = PAUSE_AFTER_SHIELD_DECISION;
        options.promptPredictedStateOnPause = PROMPT_PREDICTED_STATE_ON_PAUSE;
        options.saveInitialState = SAVE_INITIAL_STATE;
        options.renderEachStep = RENDER_EACH_STEP;
        options.rethrowOnCrash = true;
        options.printDiagnostics = true;
        options.logDir = LOG_DIR;
        options.maxDesiredSpeed = 30.0;

        options.shieldPredictFutureSeconds =  2;
        options.shieldEgoRangeMeters =  100.0;
        options.finalStabilityMode = StarkScenarioRunner.FinalStabilityMode.V3;
        options.aggressiveV3TtcThreshold = 3.5;

        options.checkChangeLaneToRearVehicleThreat = true;
        options.randomizeShieldHiddenTargetAndCooldown = true;

        StarkScenarioRunner.runShieldedScenario(realWorld, options);
    }


}
