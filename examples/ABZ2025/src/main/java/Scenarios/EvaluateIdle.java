package Scenarios;

import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.ProtectedControlledVehicle;

public class EvaluateIdle implements SafeControllerActionEvaluator {
    @Override
    public int action(ProtectedControlledVehicle ego, HighwayEngine world) {
        return 1;
    }

    @Override
    public String name() {
        return "IDLE";
    }
}
