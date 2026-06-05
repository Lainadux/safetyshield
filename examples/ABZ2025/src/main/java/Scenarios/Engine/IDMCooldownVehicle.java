package Scenarios.Engine;

public class IDMCooldownVehicle extends Vehicle {
    public double idmCooldownTimer = 0.0;
    public double idmActionStepLength = 0.1;

    public IDMCooldownVehicle() {
    }

    public IDMCooldownVehicle(Vehicle copy) {
        super(copy);
        if (copy instanceof IDMCooldownVehicle idmCooldownVehicle) {
            this.idmCooldownTimer = idmCooldownVehicle.idmCooldownTimer;
            this.idmActionStepLength = idmCooldownVehicle.idmActionStepLength;
        }
    }

    @Override
    public Vehicle deepCopySelf() {
        IDMCooldownVehicle copy = new IDMCooldownVehicle();
        copyBaseStateTo(copy);
        copy.idmCooldownTimer = this.idmCooldownTimer;
        copy.idmActionStepLength = this.idmActionStepLength;
        return copy;
    }

    @Override
    protected double computeIdmAcceleration(java.util.List<Vehicle> allVehicles) throws Exception {
        if (shouldReusePlannedIdmAcceleration()) {
            return this.plannedAcceleration;
        }
        double acceleration = super.computeIdmAcceleration(allVehicles);
        this.idmCooldownTimer = idmActionStepLength;
        return acceleration;
    }

    public boolean shouldReusePlannedIdmAcceleration() {
        if (idmActionStepLength <= 0.0 || idmCooldownTimer <= 0.0) {
            return false;
        }
        try {
            return getEngine().stepCount > 0;
        } catch (IllegalStateException e) {
            return false;
        }
    }

    @Override
    public void applyPhysics() {
        super.applyPhysics();
        this.idmCooldownTimer = Math.max(0.0, this.idmCooldownTimer - getEngine().dt);
    }
}
