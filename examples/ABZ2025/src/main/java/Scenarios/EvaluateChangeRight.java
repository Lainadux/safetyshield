package Scenarios;

import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.ProtectedControlledVehicle;

public class EvaluateChangeRight implements SafeControllerActionEvaluator {
    @Override
    public int action(ProtectedControlledVehicle ego, HighwayEngine world) {
        return 2;
    }

    @Override
    public String name() {
        return "LANE_RIGHT";
    }
}
