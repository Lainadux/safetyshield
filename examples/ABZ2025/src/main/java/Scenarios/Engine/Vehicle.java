/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *                Copyright (C) 2023.
 *
 * See the NOTICE file distributed with this work for additional information
 * regarding copyright ownership.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *             http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package Scenarios.Engine;

import com.google.gson.annotations.SerializedName;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

// this class is for HighwayEngine
public class Vehicle {

    boolean mobiling = false; //for debugging
    public int[] possible_lanes = new int[]{0, 1, 2}; // for debugging
    public double karma_a_new = 0.0; // for debugging
    public Map<Integer, List<Double>> mobil = new HashMap<>(); // for debugging
    public Map<Integer, MobilDebugInfo> mobilDebug = new HashMap<>(); // for rendering/debugging
    public String id ="default";
    public double politeness = 0.0;
    private transient HighwayEngine engine;
    public HighwayEngine getEngine() {
        if (this.engine == null) {
            throw new IllegalStateException("Engine not injected yet!");
        }
        return this.engine;
    }
    public ControlledVehicle ascendAsControlledVehicle(){
        this.id = "0";
        if (this instanceof ControlledVehicle) {
            return (ControlledVehicle) this;
        }
        else{
           // return new ControlledVehicle(this.x, this.y, this.getLaneIndex(), this.speed, this.targetSpeed);
            return new ControlledVehicle(this);
        }
    }
    public double cooldownTimer = 0.0;
    public int target_lane_index;
    public int lane_index;
    //public double starked_target_lane_index;
    //public double starked_lane_index;
    public int getTargetLaneIndex(){
//        if(this.starked) {
//            return (int) this.starked_target_lane_index;
//        }
        return this.target_lane_index;
    }
    public int getLaneIndex(){

        return this.lane_index;
    }
    public void setTargetLaneIndex(int laneIndex){

            this.target_lane_index = laneIndex;

    }
    public void setLaneIndex(int laneIndex){
        this.lane_index = laneIndex;
    }

    public String role = "NPC";

    public double x, y;
    public double vx, vy;
    public double speed;
    public double heading;
    public double plannedAcceleration = 0.0;
    public double plannedSteering = 0.0;
    public final double LENGTH = 5.0;
    public final double WHEELBASE = 5.0;
    public final double WIDTH = 2.0;
    public Vehicle(){

    }

    public static class MobilDebugInfo {
        public final int targetLane;
        public final String targetFrontVehicleId;
        public final String targetRearVehicleId;
        public final String originalRearVehicleId;
        public final double overallBenefit;
        public final double selfBenefit;
        public final double karma;
        public final double originalRearBenefit;

        public MobilDebugInfo(int targetLane,
                              String targetFrontVehicleId,
                              String targetRearVehicleId,
                              String originalRearVehicleId,
                              double overallBenefit,
                              double selfBenefit,
                              double karma,
                              double originalRearBenefit) {
            this.targetLane = targetLane;
            this.targetFrontVehicleId = targetFrontVehicleId;
            this.targetRearVehicleId = targetRearVehicleId;
            this.originalRearVehicleId = originalRearVehicleId;
            this.overallBenefit = overallBenefit;
            this.selfBenefit = selfBenefit;
            this.karma = karma;
            this.originalRearBenefit = originalRearBenefit;
        }
    }

    /**
     * @param copy
     * return a copy of the vehicle, with an null engine
     */

    public Vehicle(Vehicle copy){
        this.id = copy.id;
        this.politeness = copy.politeness;
        this.cooldownTimer = copy.cooldownTimer;
        this.setTargetLaneIndex(copy.getTargetLaneIndex());
        this.setLaneIndex(copy.getLaneIndex());
        this.x = copy.x;
        this.y = copy.y;
        this.vx = copy.vx;
        this.vy = copy.vy;
        this.speed = copy.speed;
        this.heading = copy.heading;
        this.plannedAcceleration = copy.plannedAcceleration;
        this.plannedSteering = copy.plannedSteering;
        this.targetSpeed = copy.targetSpeed;
    }

    protected void copyBaseStateTo(Vehicle copy) {
        copy.id = this.id;
        copy.politeness = this.politeness;
        copy.cooldownTimer = this.cooldownTimer;
        copy.setTargetLaneIndex(this.getTargetLaneIndex());
        copy.setLaneIndex(this.getLaneIndex());
        copy.x = this.x;
        copy.y = this.y;
        copy.vx = this.vx;
        copy.vy = this.vy;
        copy.speed = this.speed;
        copy.heading = this.heading;
        copy.plannedAcceleration = this.plannedAcceleration;
        copy.plannedSteering = this.plannedSteering;
        copy.targetSpeed = this.targetSpeed;
        copy.role = this.role;
    }

    public Vehicle deepCopySelf() {
        Vehicle copy = new Vehicle();
        copyBaseStateTo(copy);
        return copy;
    }

    public String getRole() {
        return this instanceof ControlledVehicle ? "EGO" : "NPC";
    }

    public void injectEngine(HighwayEngine engine) {

        if(!engine.vehicles.contains(this)) {
            throw new IllegalArgumentException("Vehicle must be added to the engine before injecting it!");
        }
        this.engine = engine;
    }

    @SerializedName("target_speed")
    public double targetSpeed = 25;
    public void planAction(List<Vehicle> allVehicles) throws Exception {

        //this.target_lane_index = EngineUtils.computeTargetLane(this, allVehicles, List.of(0, 1), this.engine);
        this.setTargetLaneIndex(EngineUtils.computeTargetLane(this, allVehicles, List.of(0, 1), this.engine));
        this.plannedAcceleration = computeIdmAcceleration(allVehicles);
        this.plannedSteering = EngineUtils.computeSteering(this);
    }

    protected double computeIdmAcceleration(List<Vehicle> allVehicles) throws Exception {
        return Math.min(
                EngineUtils.computeAccel(this, allVehicles, this.getTargetLaneIndex()),
                EngineUtils.computeAccel(this, allVehicles, this.getLaneIndex())
        );
    }


    public void applyPhysics() {

        this.speed += this.plannedAcceleration * engine.dt;


        double yawRate = (this.speed / WHEELBASE) * Math.tan(this.plannedSteering);
        this.heading += yawRate * engine.dt;

        this.x += this.speed * Math.cos(this.heading) * engine.dt;
        this.y += this.speed * Math.sin(this.heading) * engine.dt;

        this.vx = this.speed * Math.cos(this.heading);
        this.vy = this.speed * Math.sin(this.heading);


        this.lane_index = (int) Math.round(this.y / 4.0);
        this.cooldownTimer += engine.dt;


        if (Double.isNaN(this.speed) || Double.isNaN(this.heading)) {
            String role = this instanceof ControlledVehicle ? "EGO" : "NPC";
            throw new RuntimeException("NaN Virus Detected in Vehicle " + role +
                    "! a=" + this.plannedAcceleration + ", steer=" + this.plannedSteering);
        }

    }
    public void checkCollision() {
        List<Vehicle> environments = this.getEngine().vehicles;
        for (Vehicle other : environments) {

            if (other == this) continue;

            // 绠楄嚜宸卞拰鍒汉鐨勭粷瀵硅窛绂?
            double dx = Math.abs(this.x - other.x);
            double dy = Math.abs(this.y - other.y);

            // 鍋囪杞﹁締鏈?LENGTH 鍜?WIDTH 灞炴€?(瀹藉害璁句负 2.0)
            boolean overlapX = dx < (this.LENGTH / 2.0 + other.LENGTH / 2.0);
            boolean overlapY = dy < (this.WIDTH / 2.0 + other.WIDTH / 2.0);

            if (overlapX && overlapY) {

                // 鎶涘嚭甯︽湁鏄庣‘璐ｄ换鏂圭殑寮傚父
                String crashMsg = String.format(
                        "馃挜 鑷村懡纰版挒锛乗n鑲囦簨杞﹁締锛歔%s-%s] 鍦ㄧЩ鍔ㄥ悗涓€澶存挒涓婁簡 [%s-%s]锛乗n" +
                                "鑲囦簨杞﹀潗鏍?X:%.1f Y:%.1f | 琚挒杞﹀潗鏍?X:%.1f Y:%.1f",
                        this.role, this.id,
                        other.role, other.id,
                        this.x, this.y, other.x, other.y
                );
                if(this.engine.requireCollisionLog){
                    StateSaver.saveState(this.engine.vehicles, "ego "+ System.currentTimeMillis() + ".json");
                }
                throw new RuntimeException(crashMsg);
            }
        }
    }
    public double laneDistanceTo(Vehicle other) {
        if (other == null) return 0.0;

        return other.x - this.x;
    }
    //閲嶅啓print

    public String toString() {
//        /杈撳嚭闄や簡debug澶栫殑鎵€鏈夊睘鎬э紝 鍖呮嫭stark
        return String.format("Vehicle{id='%s', role='%s', x=%.1f, y=%.1f, lane_index=%d, target_lane_index=%d, speed=%.1f, plannedAcceleration=%.1f, plannedSteering=%.1f, " +
                        ", targetSpeed=%.1f, cooldownTimer=%.1f, politeness=%.2f, timestamp=%.2f}",
                id, role, x, y, getLaneIndex(), getTargetLaneIndex(), speed, plannedAcceleration, plannedSteering, targetSpeed, cooldownTimer, politeness, engine != null ? engine.stepCount * engine.dt : 0.0
        );
    }
    public boolean isEgo(){
        return this.role.equals("EGO");
    }


    public String getId() {
        return id;
    }

    public void setId(String id) {
        this.id = id;
    }

    public double getPoliteness() {
        return politeness;
    }

    public void setPoliteness(double politeness) {
        this.politeness = politeness;
    }

    public double getCooldownTimer() {
        return cooldownTimer;
    }

    public void setCooldownTimer(double cooldownTimer) {
        this.cooldownTimer = cooldownTimer;
    }

    public int getTarget_lane_index() {
        return target_lane_index;
    }

    public void setTarget_lane_index(int target_lane_index) {
        this.target_lane_index = target_lane_index;
    }

    public int getLane_index() {
        return lane_index;
    }

    public void setLane_index(int lane_index) {
        this.lane_index = lane_index;
    }

    public void setRole(String role) {
        this.role = role;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getVx() {
        return vx;
    }

    public void setVx(double vx) {
        this.vx = vx;
    }

    public double getVy() {
        return vy;
    }

    public void setVy(double vy) {
        this.vy = vy;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public double getPlannedAcceleration() {
        return plannedAcceleration;
    }

    public void setPlannedAcceleration(double plannedAcceleration) {
        this.plannedAcceleration = plannedAcceleration;
    }

    public double getPlannedSteering() {
        return plannedSteering;
    }

    public void setPlannedSteering(double plannedSteering) {
        this.plannedSteering = plannedSteering;
    }

    public double getLENGTH() {
        return LENGTH;
    }

    public double getWHEELBASE() {
        return WHEELBASE;
    }

    public double getWIDTH() {
        return WIDTH;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setTargetSpeed(double targetSpeed) {
        this.targetSpeed = targetSpeed;
    }
}

