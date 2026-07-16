package Scenarios;

import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.ProtectedControlledVehicle;

public class EvaluateFaster implements SafeControllerActionEvaluator {
    @Override
    public int action(ProtectedControlledVehicle ego, HighwayEngine world) {
        return 3;
    }

    @Override
    public String name() {
        return "FASTER";
    }
}
