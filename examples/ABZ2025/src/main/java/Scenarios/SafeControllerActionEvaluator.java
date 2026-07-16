package Scenarios;

import Scenarios.Engine.HighwayEngine;
import Scenarios.Engine.ProtectedControlledVehicle;

public interface SafeControllerActionEvaluator {
    int action(ProtectedControlledVehicle ego, HighwayEngine world);

    String name();
}
