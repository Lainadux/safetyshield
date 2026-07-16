package Scenarios;

import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.ProtectedControlledVehicle;

public class EvaluateChangeLeft implements SafeControllerActionEvaluator {
    @Override
    public int action(ProtectedControlledVehicle ego, HighwayEngine world) {
        return 0;
    }

    @Override
    public String name() {
        return "LANE_LEFT";
    }
}
