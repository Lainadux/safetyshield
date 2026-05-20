package Scenarios.Engine;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class HighwayEngine {
    public boolean egoCentered = false;
    public boolean saveInitStateAnyWay = false;
    public boolean requireRender = true;
    public boolean requireCollisionLog = false;
    public List<Vehicle> initialVehiclesStates = new ArrayList<>();
    public boolean enhancedCollisionCheckEnabled =false;
    public List<Vehicle> vehicles = new ArrayList<>();
    public double dt;
    public boolean hasEgo = false;
    public double runTime = 0.0;
    public long stepCount = 0;
    public  int STEPS_PER_SECOND;
    private JFrame frame;
    private JPanel renderPanel;
    private final int SCALE = 15;
    private final int LANE_WIDTH = 4;
    public int numLanes = 3;


    public ControlledVehicle getEgoVehicle() {
        for (Vehicle v : vehicles) {
            if (v instanceof ControlledVehicle) {
                return (ControlledVehicle) v;
            }
        }
        throw new IllegalStateException("No EGO vehicle found in the engine!");
    }
    public void setEgoVehicle(ControlledVehicle ego) {
        // Remove existing EGO if present
        vehicles.removeIf(v -> v instanceof ControlledVehicle);
        // Add the new EGO vehicle
        this.addVehicle(ego);
        ego.injectEngine(this);
    }
    public HighwayEngine(){

    }
    public HighwayEngine(double dt, boolean hasEgo) {
        this.hasEgo = hasEgo;
        this.dt = dt;
        this.STEPS_PER_SECOND = (int) Math.round(1.0 / dt);
    }
    public HighwayEngine(double dt, boolean hasEgo, int numLanes) {
        this.hasEgo = hasEgo;
        this.dt = dt;
        this.STEPS_PER_SECOND = (int) Math.round(1.0 / dt);
        this.numLanes = numLanes;
    }
    public HighwayEngine(double dt, boolean hasEgo,boolean enhancedCollisionCheckEnabled, List<Vehicle> initialVehicles) {
        for(Vehicle v : initialVehicles){
            initialVehiclesStates.add(v.deepCopySelf());
        }
        this.enhancedCollisionCheckEnabled = enhancedCollisionCheckEnabled;
        this.hasEgo = hasEgo;
        this.dt = dt;
        this.STEPS_PER_SECOND = (int) Math.round(1.0 / dt);
        for (Vehicle v : initialVehicles) {
            this.addVehicle(v);
            v.injectEngine(this);
        }
    }
    public HighwayEngine(double dt) {
        this.hasEgo = false;
        this.dt = dt;
        this.STEPS_PER_SECOND = (int) Math.round(1.0 / dt);
    }
    public int[] computePossibleLanes(Vehicle vehicle){
        List<Integer> possibleLanes = new ArrayList<>();
        int currentLane = vehicle.getLaneIndex();
        for (int i = currentLane - 1; i <= currentLane + 1; i++) {
            if (i >= 0 && i < numLanes) {
                possibleLanes.add(i);
            }
        }
        return possibleLanes.stream().mapToInt(i -> i).toArray();

    }

    public void addVehicle(Vehicle v) {
        this.vehicles.add(v);
    }




    public void step() throws Exception {
        if(EngineUtils.isDoubleEqual(this.runTime, 0.0)){
            for(Vehicle v : vehicles) {
                this.initialVehiclesStates.add(v.deepCopySelf());
            }
            if (saveInitStateAnyWay) {
                StateSaver.saveState(this.initialVehiclesStates, "initial_state_" + System.currentTimeMillis() + ".json");
            }
        }

        for (Vehicle v : vehicles) {
            v.planAction(vehicles);
        }
        for (Vehicle v : vehicles) {
            v.applyPhysics();
        }


        // 璋冭瘯鎵撳嵃锛氬彧鐪?Ego 杞?
        Vehicle ego = vehicles.get(0);
        this.runTime += this.dt;
        stepCount++;
        if(hasEgo) {
            for (Vehicle v : vehicles) {
                if (v instanceof ControlledVehicle) {
                    v.checkCollision();
                }
            }
            if(enhancedCollisionCheckEnabled) {
                this.checkCollisions();
            }
        }
        else{
            this.checkCollisions();
        }


    }
    public void step(int targetLaneIndex, double targetSpeed) throws Exception {
        if(EngineUtils.isDoubleEqual(this.runTime, 0.0)){
            for(Vehicle v : vehicles) {
                this.initialVehiclesStates.add(v.deepCopySelf());
            }
            if (saveInitStateAnyWay) {
                StateSaver.saveState(this.initialVehiclesStates, "initial_state_" + System.currentTimeMillis() + ".json");
            }
        }

        for (Vehicle v : vehicles) {
            v.planAction(vehicles);
            if(v instanceof ControlledVehicle) {
                v.setTargetLaneIndex(v.getLaneIndex());
                v.targetSpeed = targetSpeed;
            }
        }
        for (Vehicle v : vehicles) {
            v.applyPhysics();
        }


        // 璋冭瘯鎵撳嵃锛氬彧鐪?Ego 杞?
        Vehicle ego = vehicles.get(0);
        this.runTime += this.dt;
        stepCount++;
        if(hasEgo) {
            for (Vehicle v : vehicles) {
                if (v instanceof ControlledVehicle) {
                    v.checkCollision();
                }
            }
            if(enhancedCollisionCheckEnabled) {
                this.checkCollisions();
            }
        }
        else{
            this.checkCollisions();
        }
    }
    public void populateTraffic(int targetVehicles, int numLanes, double minX, double maxX) {
        this.numLanes = numLanes;
        Random rand = new Random();
        int spawned = 0;
        int attempts = 0;
        int MAX_ATTEMPTS = targetVehicles * 20;


        final double LANE_WIDTH = 4.0;
        final double SAFE_SPAWN_DISTANCE = 15.0;



        while (spawned < targetVehicles && attempts < MAX_ATTEMPTS) {
            attempts++;


            int lane = rand.nextInt(numLanes);
            double yCenter = lane * LANE_WIDTH;


            double x = minX + (maxX - minX) * rand.nextDouble();

            boolean collision = false;
            for (Vehicle existing : this.vehicles) {
//                if (existing.lane_index == lane) {
                if(existing.getLaneIndex() == lane) {
                    if (Math.abs(existing.x - x) < SAFE_SPAWN_DISTANCE) {
                        collision = true;
                        break;
                    }
                }
            }


            if (collision) continue;

            Vehicle v;
            if (hasEgo && spawned == 0) {
                v = new ControlledVehicle();
                v.role = "EGO";

            }
            else {
                v = new Vehicle();
                v.role = "NPC";
            }


            v.x = x;
            v.y = yCenter;
           // v.lane_index = lane;
            //v.target_lane_index = lane;
            v.setLaneIndex(lane);
            v.setTargetLaneIndex(lane);

            v.id = ""+spawned;
            v.cooldownTimer = rand.nextDouble() * 1;

            double speed = 20.0 + rand.nextGaussian() * 3.0;


            v.speed = Math.max(10.0, Math.min(30.0, speed));
            if(egoCentered && v.role.equals("EGO")){
                lane = numLanes/2;
                v.y = lane * LANE_WIDTH;
                v.setLaneIndex(lane);
                v.setTargetLaneIndex(lane);
                v.speed = 25;
            }


            v.targetSpeed = v.speed + rand.nextDouble() * 5.0;

            this.addVehicle(v);
            v.injectEngine(this);




            spawned++;
        }


    }

    public void populateProtectedTraffic(int targetVehicles, int numLanes, double minX, double maxX) {
        this.numLanes = numLanes;
        Random rand = new Random();
        int spawned = 0;
        int attempts = 0;
        int MAX_ATTEMPTS = targetVehicles * 20;


        final double LANE_WIDTH = 4.0;
        final double SAFE_SPAWN_DISTANCE = 15.0;



        while (spawned < targetVehicles && attempts < MAX_ATTEMPTS) {
            attempts++;


            int lane = rand.nextInt(numLanes);
            double yCenter = lane * LANE_WIDTH;


            double x = minX + (maxX - minX) * rand.nextDouble();

            boolean collision = false;
            for (Vehicle existing : this.vehicles) {
//                if (existing.lane_index == lane) {
                if(existing.getLaneIndex() == lane) {
                    if (Math.abs(existing.x - x) < SAFE_SPAWN_DISTANCE) {
                        collision = true;
                        break;
                    }
                }
            }


            if (collision) continue;

            Vehicle v;
            if (hasEgo && spawned == 0) {
                v = new ProtectedControlledVehicle();
                v.role = "EGO";

            }
            else {
                v = new Vehicle();
                v.role = "NPC";
            }


            v.x = x;
            v.y = yCenter;
            // v.lane_index = lane;
            //v.target_lane_index = lane;
            v.setLaneIndex(lane);
            v.setTargetLaneIndex(lane);

            v.id = ""+spawned;
            v.cooldownTimer = rand.nextDouble() * 1;

            double speed = 20.0 + rand.nextGaussian() * 3.0;


            v.speed = Math.max(10.0, Math.min(30.0, speed));
            if(egoCentered && v.role.equals("EGO")){
                lane = numLanes/2;
                v.y = lane * LANE_WIDTH;
                v.setLaneIndex(lane);
                v.setTargetLaneIndex(lane);
                v.speed = 25;
            }


            v.targetSpeed = v.speed + rand.nextDouble() * 5.0;

            this.addVehicle(v);
            v.injectEngine(this);




            spawned++;
        }


    }

    public void render() {


        if (frame == null) {
            initUI();
        }


        renderPanel.repaint();


        try {
            Thread.sleep((long) (dt * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void initUI() {
        frame = new JFrame("STARK Highway Simulator");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(1200, 400);

        renderPanel = new JPanel() {
            @Override
            protected void paintComponent(Graphics g) {
                super.paintComponent(g);
                drawHighway((Graphics2D) g);
            }
        };

        renderPanel.setBackground(new Color(40, 40, 40));
        frame.add(renderPanel);
        frame.setVisible(true);
    }


    private void drawHighway(Graphics2D g2d) {

        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        int screenWidth = renderPanel.getWidth();
        int screenHeight = renderPanel.getHeight();
        int topMargin = 100;


        double cameraX = vehicles.isEmpty() ? 0 : getCameraVehicle().x;
        int offsetX = (int) (screenWidth / 2 - cameraX * SCALE); // 鎶婅溅鏀惧湪灞忓箷宸︿晶 1/3 澶?

        g2d.setColor(Color.WHITE);


        double[] lineYPositions = {-2.0, 2.0, 6.0, 10.0};

        for (int i = 0; i < lineYPositions.length; i++) {

            int yPixel = topMargin + (int)(lineYPositions[i] * SCALE);

            if (i == 0 || i == lineYPositions.length - 1) {

                g2d.setStroke(new BasicStroke(3));
            } else {

                float[] dash = {15.0f};
                g2d.setStroke(new BasicStroke(2, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10.0f, dash, 0.0f));
            }


            g2d.drawLine(0, yPixel, screenWidth, yPixel);
        }


        for (Vehicle v : vehicles) {

            int px = (int) (v.x * SCALE) + offsetX;
            int py = (int) (v.y * SCALE) + topMargin;

            int carPixelLength = (int) (v.LENGTH * SCALE);
            int carPixelWidth = (int) (v.WIDTH * SCALE);


            AffineTransform oldTransform = g2d.getTransform();


            g2d.translate(px, py);

            g2d.rotate(v.heading);

            if (v instanceof ControlledVehicle) {
                g2d.setColor(new Color(0, 200, 255));
            } else {
                g2d.setColor(new Color(255, 80, 80));
            }


            g2d.fillRect(-carPixelLength / 2, -carPixelWidth / 2, carPixelLength, carPixelWidth);


            g2d.setColor(Color.BLACK);
            g2d.fillRect(carPixelLength / 4, -carPixelWidth / 2, 4, carPixelWidth);

            g2d.setTransform(oldTransform);


            g2d.setColor(Color.YELLOW);
            //g2d.drawString(String.format("V:%.1f L:%d", v.speed, v.lane_index), px - 15, py - 20);
            g2d.drawString(String.format("V:%.1f L:%d", v.speed, v.getLaneIndex()), px - 15, py - 20);

            g2d.drawString(String.format("(vx:%.1f, vy:%.1f)", v.x, v.y), px - 15, py + 30);

            g2d.drawString(String.format("A:%.2f", v.plannedAcceleration), px - 15, py + 50);

            g2d.drawString(String.format("T:%.1f", v.targetSpeed), px - 15, py + 70);


//            for (Integer lane : v.mobil.keySet()) {
//                List<Double> mobilValues = v.mobil.get(lane);
//                g2d.drawString(String.format("lane:%d: overall:%.2f,self:%.2f,karmanew%.2f", lane, mobilValues.get(0), mobilValues.get(1), mobilValues.get(3)), px - 15, py + 90 + lane * 20);
//            }

            //cooldowntimer
            g2d.drawString(String.format("cool:%.1f", v.cooldownTimer), px - 15, py + 150);
            //mobiling
            g2d.drawString(String.format("mobiling:%b", v.mobiling), px - 15, py + 170);
            //possiblelanes
            for(int i =0; i<v.possible_lanes.length; i++) {
                g2d.drawString(String.format("possible lane:%d", v.possible_lanes[i]), px - 15, py + 190 + i * 20);
            }

            g2d.drawString(String.format("id:%s", v.id), px - 15, py);
            //karma_a_new
            //g2d.drawString(String.format("karma_a_new:%.2f", v.karma_a_new), px - 15, py + 250);
        }
    }

    private Vehicle getCameraVehicle() {
        Vehicle cameraVehicle = vehicles.get(0);
        int cameraId = parseVehicleId(cameraVehicle.id);
        for (Vehicle vehicle : vehicles) {
            int vehicleId = parseVehicleId(vehicle.id);
            if (vehicleId < cameraId) {
                cameraVehicle = vehicle;
                cameraId = vehicleId;
            }
        }
        return cameraVehicle;
    }

    private int parseVehicleId(String id) {
        try {
            return Integer.parseInt(id);
        } catch (NumberFormatException e) {
            return Integer.MAX_VALUE;
        }
    }
   public void checkCollisions() {

        for (int i = 0; i < vehicles.size(); i++) {
            for (int j = i + 1; j < vehicles.size(); j++) {
                Vehicle v1 = vehicles.get(i);
                Vehicle v2 = vehicles.get(j);


                double dx = Math.abs(v1.x - v2.x);
                double dy = Math.abs(v1.y - v2.y);


                boolean overlapX = dx < (v1.LENGTH / 2.0 + v2.LENGTH / 2.0);
                boolean overlapY = dy < (2.0 / 2.0 + 2.0 / 2.0); // 鍋囪 WIDTH 鏄?2.0

                if (overlapX && overlapY) {

                    String crashMsg = String.format(
                            "馃挜 鑷村懡鐗╃悊纰版挒妫€娴嬭Е鍙戯紒\n" +
                                    "鑲囦簨杞﹁締锛歔%s-%s] 涓?[%s-%s] 鍙戠敓浜嗛噸鍙狅紒\n" +
                                    "鎺ヨЕ鐐瑰潗鏍?-> V1 X:%.1f Y:%.1f | V2 X:%.1f Y:%.1f",
                            v1.getRole(), v1.id,
                            v2.getRole(), v2.id,
                            v1.x, v1.y, v2.x, v2.y
                    );
                    if (requireCollisionLog) {
                        System.err.println(crashMsg);
                        StateSaver.saveState(this.vehicles, "engine "+System.currentTimeMillis() + ".json");
                    }
                    throw new RuntimeException(crashMsg);
                }
            }
        }
    }

    public StarkShieldApp createStarkShieldApp(int futureSeconds) {
        if(this.vehicles == null || this.vehicles.isEmpty()) {
            throw new IllegalStateException("Engine must have vehicles to create StarkShieldApp");
        }

        return new StarkShieldApp(this, this.vehicles, futureSeconds);
    }


}
