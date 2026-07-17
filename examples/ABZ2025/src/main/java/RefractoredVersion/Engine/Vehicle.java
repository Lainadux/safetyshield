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

package RefractoredVersion.Engine;


import Scenarios.Engine.HighwayEngine;
import lombok.Getter;
import lombok.Setter;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/**
 * Represents a vehicle in the traffic simulation.
 * Contains properties for vehicle dynamics, position, and behavior.
 */
@Getter
@Setter
public class Vehicle {
//    TAU_ACC = 0.6  # [s]
//    TAU_HEADING = 0.2  # [s]
//    TAU_LATERAL = 0.6  # [s]
//
//    TAU_PURSUIT = 0.5 * TAU_HEADING  # [s]
//    KP_A = 1 / TAU_ACC
//            KP_HEADING = 1 / TAU_HEADING
//    KP_LATERAL = 1 / TAU_LATERAL  # [1/s]
//    MAX_STEERING_ANGLE = np.pi / 3  # [rad]
//    DELTA_SPEED = 5  # [m/s]

    public double TAU_ACC = 0.6;
    public double TAU_HEADING = 0.2;
    public double TAU_LATERAL = 0.6;
    public double TAU_PURSUIT = 0.5 * TAU_HEADING;
    public double KP_A = 1 / TAU_ACC;
    public double KP_HEADING = 1 / TAU_HEADING;
    public double KP_LATERAL = 1 / TAU_LATERAL;
    public double MAX_STEERING_ANGLE = Math.PI / 3;
    public double DELTA_SPEED = 5.0;

    //public Consumer<Boolean> handleCollision;
    //public boolean RequireAccurateCollisionCheck = false;



    // Debugging variables
    boolean mobiling = false; //for debugging
    public int[] possible_lanes = new int[]{0, 1, 2}; // for debugging
    public double karma_a_new = 0.0; // for debugging
    public Map<Integer, List<Double>> mobil = new HashMap<>(); // for debugging


    // Vehicle properties
    public double targetSpeed; // Desired speed of the vehicle


    public String id ="default"; // Unique identifier for the vehicle
    public double politeness = 0.0; // Politeness factor in interactions
    private transient JavaHighwayEngine engine; // Reference to the simulation engine

    //the engine is used to let this vehicle know the current state of the world
    public JavaHighwayEngine getEngine() {

        if (this.engine == null) {
            throw new IllegalStateException("Engine not injected yet!");
        }
        return this.engine;
    }


    public double cooldownTimer = 0.0;
    public int targetLaneIndex;
    public int laneIndex;
    public String role = "NPC";
    public double x, y;
    public double vx, vy;
    public double speed;
    public transient double previousSecondSpeed = Double.NaN;
    public double heading;
    public double plannedAcceleration = 0.0;
    public double plannedSteering = 0.0;
    public final double LENGTH = 5.0;
    public final double WIDTH = 2.0;
    public final double diagonal = Math.sqrt(Math.pow(LENGTH, 2) + Math.pow(WIDTH, 2));

    public void applyPhysics(){
        double deltaF = this.plannedSteering;
        //beta = arctan(1 / 2 * np.tan(delta_f))
        double beta  = Math.atan((double) 1 / 2 * Math.tan(deltaF));
        this.speed += this.plannedAcceleration * engine.getDt();
        this.speed = Math.max(0.0, this.speed);
        this.vx = this.speed * Math.cos(this.heading + beta);
        this.vy = this.speed * Math.sin(this.heading + beta);
        this.x += this.vx * engine.getDt();
        this.y += this.vy * engine.getDt();
        //  self.heading += self.speed * np.sin(beta) / (self.LENGTH / 2) * dt
        this.heading += this.speed * Math.sin(beta) / (this.LENGTH / 2) * engine.getDt();
        this.setLaneIndex(Math.max(0, Math.min(engine.numLanes - 1, (int) Math.round(this.y / 4.0))));
        this.cooldownTimer += engine.getDt();
    }
    public double laneDistanceTo(Vehicle other) {
        if (other == null) return 0.0;

        return other.x - this.x;
    }
  public void planAction(List<Vehicle> allVehicles) throws Exception {

        //this.target_lane_index = EngineUtils.computeTargetLane(this, allVehicles, List.of(0, 1), this.engine);
        this.setTargetLaneIndex(JavaHighwayEngineUtils.computeTargetLane(this, allVehicles, List.of(0, 1), this.engine));
        this.plannedAcceleration = JavaHighwayEngineUtils.computeIdmAcceleration(this, allVehicles);
        this.plannedSteering = JavaHighwayEngineUtils.computeSteering(this);
    }


    public void checkCollision(Consumer<Boolean> handleCollision) {
        List<Vehicle> environments = this.getEngine().vehicles;
        for (Vehicle other : environments) {
            if(other == this) continue;
            else{
//
             double centerDistance = Math.sqrt(Math.pow(other.x - this.x, 2) + Math.pow(other.y - this.y, 2));
             double c = (this.diagonal + other.diagonal) / 2 + this.speed * this.getEngine().getDt();
             if (centerDistance > c) {
                    continue;
             }
             else {

                    double dx = Math.abs(other.x - this.x);
                    double dy = Math.abs(other.y - this.y);
                    boolean colliding =
                            dx < (this.LENGTH + other.LENGTH) / 2.0
                                    && dy < (this.WIDTH + other.WIDTH) / 2.0;
                    handleCollision.accept(colliding);
                }

            }
        }
    }
    public void checkCollision() {
        List<Vehicle> environments = this.getEngine().vehicles;
        for (Vehicle other : environments) {
            if(other == this) continue;
            else{
//
                double centerDistance = Math.sqrt(Math.pow(other.x - this.x, 2) + Math.pow(other.y - this.y, 2));
                double c = (this.diagonal + other.diagonal) / 2 + this.speed * this.getEngine().getDt();
                if (centerDistance > c) {
                    continue;
                }
                else {

                    double dx = Math.abs(other.x - this.x);
                    double dy = Math.abs(other.y - this.y);
                    boolean colliding =
                            dx < (this.LENGTH + other.LENGTH) / 2.0
                                    && dy < (this.WIDTH + other.WIDTH) / 2.0;
                    if (colliding) {
                        throw new RuntimeException(String.format(
                                "A collision has been detected between vehicle %s(%s) and vehicle %s(%s). "
                                        + "v1=(%.2f, %.2f), v2=(%.2f, %.2f)",
                                this.id, this.role,
                                other.id, other.role,
                                this.x, this.y,
                                other.x, other.y
                        ));
                    }
                }

            }
        }
    }




}
