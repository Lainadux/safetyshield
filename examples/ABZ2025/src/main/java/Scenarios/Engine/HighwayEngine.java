package Scenarios.Engine;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.AffineTransform;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

public class HighwayEngine {
    public enum NpcVehicleType {
        DEFAULT,
        IDM_COOLDOWN,
        DELAYED_IDM
    }

    public boolean egoCentered = false;
    public boolean placeEgoAtTrafficMiddle = false;
    public boolean saveInitStateAnyWay = false;
    public boolean requireRender = true;
    public boolean requireCollisionLog = false;
    public boolean continueAfterNpcCollision = false;
    public List<Vehicle> initialVehiclesStates = new ArrayList<>();
    public boolean enhancedCollisionCheckEnabled =false;
    public List<Vehicle> vehicles = new ArrayList<>();
    private volatile boolean predictedStatePromptEnabled = false;
    private volatile boolean renderDebugEnabled = false;
    private volatile Runnable predictedStateQueryAction;
    private volatile Runnable showIfAction;
    public double dt;
    public boolean hasEgo = false;
    public double runTime = 0.0;
    public long stepCount = 0;
    public  int STEPS_PER_SECOND;
    private JFrame frame;
    private JPanel renderPanel;
    private JButton predictedStateQueryButton;
    private JButton showIfButton;
    private static final double DEFAULT_RENDER_SCALE = 7.0;
    private static final double MIN_RENDER_SCALE = 3.0;
    private static final double MAX_RENDER_SCALE = 25.0;
    private double renderScale = DEFAULT_RENDER_SCALE;
    private final int LANE_WIDTH = 4;
    private int cameraDragOffsetX = 0;
    private int cameraDragOffsetY = 0;
    private Point lastCameraDragPoint;
    private static final double MIN_SPAWN_FRONT_GAP = 12.0;
    private static final double SPAWN_REACTION_TIME = 0.5;
    private static final double SPAWN_MAX_BRAKE = 5.0;
    private static final int SPAWN_ATTEMPT_MULTIPLIER = 200;
    private static final double POLITENESS_MEAN = 0.5;
    private static final double POLITENESS_STD = 1.0 / 6.0;
    public NpcVehicleType npcVehicleType = NpcVehicleType.DEFAULT;
    public double npcIdmActionStepLength = 0.1;
    public double npcReactionDelay = 0.3;
    public double idmTimeWanted = EngineUtils.DEFAULT_TIME_WANTED;
    public double npcInitialSpeedMin = 21.0;
    public double npcInitialSpeedMax = 24.0;
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

    public boolean isPredictedStatePromptEnabled() {
        return predictedStatePromptEnabled;
    }

    public void setPredictedStateQueryAction(Runnable action) {
        this.predictedStateQueryAction = action;
        if (predictedStateQueryButton != null) {
            SwingUtilities.invokeLater(() -> predictedStateQueryButton.setEnabled(action != null));
        }
    }

    public void setShowIfAction(Runnable action) {
        this.showIfAction = action;
        if (showIfButton != null) {
            SwingUtilities.invokeLater(() -> showIfButton.setEnabled(action != null));
        }
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
        populateTraffic(targetVehicles, numLanes, minX, maxX, false);
    }

    public void populateTraffic(int targetVehicles, int numLanes, double minX, double maxX, boolean polite) {
        this.numLanes = numLanes;
        Random rand = new Random();
        int spawned = 0;
        int attempts = 0;
        int MAX_ATTEMPTS = targetVehicles * SPAWN_ATTEMPT_MULTIPLIER;

        final double LANE_WIDTH = 4.0;



        while (spawned < targetVehicles && attempts < MAX_ATTEMPTS) {
            attempts++;


            int lane = rand.nextInt(numLanes);
            double yCenter = lane * LANE_WIDTH;


            Vehicle v;
            if (hasEgo && spawned == 0) {
                v = new ControlledVehicle();
                v.role = "EGO";

            }
            else {
                v = createNpcVehicle();
                v.role = "NPC";
            }


            v.y = yCenter;
           // v.lane_index = lane;
            //v.target_lane_index = lane;
            v.setLaneIndex(lane);
            v.setTargetLaneIndex(lane);

            v.id = ""+spawned;
            if (polite) {
                v.politeness = clippedGaussian(rand, POLITENESS_MEAN, POLITENESS_STD, 0.0, 1.0);
            }
            v.cooldownTimer = rand.nextDouble() * 1;

            double speed = sampleNpcInitialSpeed(rand);


            v.speed = speed;
            if (v.role.equals("EGO")) {
                v.speed = 25;
            }
            v.x = "EGO".equals(v.role) ? 0.0 : nextPythonStyleNpcX(rand, v.speed);
            if(egoCentered && v.role.equals("EGO")){
                lane = numLanes/2;
                v.y = lane * LANE_WIDTH;
                v.setLaneIndex(lane);
                v.setTargetLaneIndex(lane);
            }

            v.vx = v.speed;
            v.vy = 0.0;

            v.targetSpeed = v.speed;

            if (!isSpawnDynamicallySafe(v)) {
                continue;
            }

            this.addVehicle(v);
            v.injectEngine(this);




            spawned++;
        }


    }

    private double clippedGaussian(Random rand, double mean, double std, double min, double max) {
        double value = mean + rand.nextGaussian() * std;
        return Math.max(min, Math.min(max, value));
    }

    public void populateProtectedTraffic(int targetVehicles, int numLanes, double minX, double maxX) {
        this.numLanes = numLanes;
        Random rand = new Random();
        int spawned = 0;
        int attempts = 0;
        int MAX_ATTEMPTS = targetVehicles * SPAWN_ATTEMPT_MULTIPLIER;

        final double LANE_WIDTH = 4.0;



        while (spawned < targetVehicles && attempts < MAX_ATTEMPTS) {
            attempts++;


            int lane = rand.nextInt(numLanes);
            double yCenter = lane * LANE_WIDTH;

            Vehicle v;
            if (hasEgo && spawned == 0) {
                v = new ProtectedControlledVehicle();
                v.role = "EGO";

            }
            else {
                v = createNpcVehicle();
                v.role = "NPC";
            }


            v.y = yCenter;
            // v.lane_index = lane;
            //v.target_lane_index = lane;
            v.setLaneIndex(lane);
            v.setTargetLaneIndex(lane);

            v.id = ""+spawned;
            v.cooldownTimer = rand.nextDouble() * 1;

            double speed = sampleNpcInitialSpeed(rand);


            v.speed = speed;
            if (v.role.equals("EGO")) {
                v.speed = 25;
            }
            v.x = "EGO".equals(v.role) ? 0.0 : nextPythonStyleNpcX(rand, v.speed);
            if(egoCentered && v.role.equals("EGO")){
                lane = numLanes/2;
                v.y = lane * LANE_WIDTH;
                v.setLaneIndex(lane);
                v.setTargetLaneIndex(lane);
            }

            v.vx = v.speed;
            v.vy = 0.0;

            v.targetSpeed = v.speed;

            if (!isSpawnDynamicallySafe(v)) {
                continue;
            }

            this.addVehicle(v);
            v.injectEngine(this);




            spawned++;
        }


    }

    private double nextPythonStyleNpcX(Random rand, double speed) {
        double defaultSpacing = 12.0 + speed;
        double offset = defaultSpacing * Math.exp(-5.0 / 40.0 * this.numLanes);
        double x0 = this.vehicles.isEmpty() ? 3.0 * offset : maxVehicleX();
        return x0 + offset * (0.9 + 0.2 * rand.nextDouble());
    }

    private double sampleNpcInitialSpeed(Random rand) {
        double minSpeed = Math.min(npcInitialSpeedMin, npcInitialSpeedMax);
        double maxSpeed = Math.max(npcInitialSpeedMin, npcInitialSpeedMax);
        return minSpeed + rand.nextDouble() * (maxSpeed - minSpeed);
    }

    private double maxVehicleX() {
        double maxX = Double.NEGATIVE_INFINITY;
        for (Vehicle vehicle : this.vehicles) {
            maxX = Math.max(maxX, vehicle.x);
        }
        return maxX;
    }

    private Vehicle createNpcVehicle() {
        if (npcVehicleType == NpcVehicleType.IDM_COOLDOWN) {
            IDMCooldownVehicle vehicle = new IDMCooldownVehicle();
            vehicle.idmActionStepLength = npcIdmActionStepLength;
            vehicle.idmCooldownTimer = 0.0;
            return vehicle;
        }
        if (npcVehicleType == NpcVehicleType.DELAYED_IDM) {
            DelayedIDMVehicle vehicle = new DelayedIDMVehicle();
            vehicle.idmActionStepLength = npcIdmActionStepLength;
            vehicle.idmCooldownTimer = 0.0;
            vehicle.reactionDelay = npcReactionDelay;
            return vehicle;
        }
        return new Vehicle();
    }

    private boolean isSpawnDynamicallySafe(Vehicle candidate) {
        for (Vehicle existing : this.vehicles) {
            if (existing.getLaneIndex() != candidate.getLaneIndex()) {
                continue;
            }

            Vehicle rear = candidate.x < existing.x ? candidate : existing;
            Vehicle front = candidate.x < existing.x ? existing : candidate;
            double bumperGap = front.x - rear.x - rear.LENGTH;
            if (bumperGap < requiredInitialBumperGap(rear, front)) {
                return false;
            }
        }
        return true;
    }

    private double requiredInitialBumperGap(Vehicle rear, Vehicle front) {
        double rearSpeed = Math.max(0.0, rear.speed);
        double frontSpeed = Math.max(0.0, front.speed);
        double reactionDistance = rearSpeed * SPAWN_REACTION_TIME;
        double brakingDifference = (rearSpeed * rearSpeed - frontSpeed * frontSpeed) / (2.0 * SPAWN_MAX_BRAKE);
        return MIN_SPAWN_FRONT_GAP + reactionDistance + Math.max(0.0, brakingDifference);
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
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        frame.setSize(1200, 400);
        frame.setLayout(new BorderLayout());

        renderPanel = new JPanel() {
            @Override
            protected void paintComponent(Graphics g) {
                super.paintComponent(g);
                drawHighway((Graphics2D) g);
            }
        };

        renderPanel.setBackground(new Color(40, 40, 40));
        setupCameraDrag();
        JToggleButton predictionPromptButton = new JToggleButton("Prediction query: OFF");
        predictionPromptButton.addActionListener(e -> {
            predictedStatePromptEnabled = predictionPromptButton.isSelected();
            predictionPromptButton.setText(predictedStatePromptEnabled
                    ? "Prediction query: ON"
                    : "Prediction query: OFF");
        });
        predictedStateQueryButton = new JButton("Query vehicle");
        predictedStateQueryButton.setEnabled(false);
        predictedStateQueryButton.addActionListener(e -> {
            Runnable action = predictedStateQueryAction;
            if (action != null) {
                action.run();
            }
        });
        showIfButton = new JButton("Show if");
        showIfButton.setEnabled(false);
        showIfButton.addActionListener(e -> {
            Runnable action = showIfAction;
            if (action != null) {
                action.run();
            }
        });
        JPanel toolbar = new JPanel(new FlowLayout(FlowLayout.LEFT));
        toolbar.add(predictionPromptButton);
        toolbar.add(predictedStateQueryButton);
        toolbar.add(showIfButton);
        JButton zoomOutButton = new JButton("Zoom -");
        zoomOutButton.addActionListener(e -> changeRenderScale(1.0 / 1.25));
        JButton zoomInButton = new JButton("Zoom +");
        zoomInButton.addActionListener(e -> changeRenderScale(1.25));
        JButton resetZoomButton = new JButton("Reset zoom");
        resetZoomButton.addActionListener(e -> {
            renderScale = DEFAULT_RENDER_SCALE;
            cameraDragOffsetX = 0;
            cameraDragOffsetY = 0;
            renderPanel.repaint();
        });
        toolbar.add(zoomOutButton);
        toolbar.add(zoomInButton);
        toolbar.add(resetZoomButton);
        JToggleButton debugButton = new JToggleButton("Debug: OFF");
        debugButton.addActionListener(e -> {
            renderDebugEnabled = debugButton.isSelected();
            debugButton.setText(renderDebugEnabled ? "Debug: ON" : "Debug: OFF");
            renderPanel.repaint();
        });
        toolbar.add(debugButton);

        frame.add(toolbar, BorderLayout.NORTH);
        frame.add(renderPanel, BorderLayout.CENTER);
        frame.setVisible(true);
    }

    private void setupCameraDrag() {
        MouseAdapter cameraDragHandler = new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                if (SwingUtilities.isLeftMouseButton(e)) {
                    lastCameraDragPoint = e.getPoint();
                    renderPanel.setCursor(Cursor.getPredefinedCursor(Cursor.MOVE_CURSOR));
                }
            }

            @Override
            public void mouseDragged(MouseEvent e) {
                if (lastCameraDragPoint == null) {
                    return;
                }
                Point currentPoint = e.getPoint();
                cameraDragOffsetX += currentPoint.x - lastCameraDragPoint.x;
                cameraDragOffsetY += currentPoint.y - lastCameraDragPoint.y;
                lastCameraDragPoint = currentPoint;
                renderPanel.repaint();
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                lastCameraDragPoint = null;
                renderPanel.setCursor(Cursor.getDefaultCursor());
            }

            @Override
            public void mouseClicked(MouseEvent e) {
                if (e.getClickCount() == 2 && SwingUtilities.isLeftMouseButton(e)) {
                    cameraDragOffsetX = 0;
                    cameraDragOffsetY = 0;
                    renderPanel.repaint();
                }
            }
        };
        renderPanel.addMouseListener(cameraDragHandler);
        renderPanel.addMouseMotionListener(cameraDragHandler);
        renderPanel.addMouseWheelListener(e -> {
            double factor = e.getWheelRotation() < 0 ? 1.15 : 1.0 / 1.15;
            changeRenderScale(factor);
        });
    }

    private void changeRenderScale(double factor) {
        double nextScale = renderScale * factor;
        renderScale = Math.max(MIN_RENDER_SCALE, Math.min(MAX_RENDER_SCALE, nextScale));
        renderPanel.repaint();
    }

    private void drawHighway(Graphics2D g2d) {

        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        int screenWidth = renderPanel.getWidth();
        int screenHeight = renderPanel.getHeight();
        int topMargin = 100;


        double cameraX = vehicles.isEmpty() ? 0 : getCameraVehicle().x;
        int offsetX = (int) (screenWidth / 2 - cameraX * renderScale) + cameraDragOffsetX;
        int offsetY = cameraDragOffsetY;

        g2d.setColor(Color.WHITE);


        double[] lineYPositions = {-2.0, 2.0, 6.0, 10.0};

        for (int i = 0; i < lineYPositions.length; i++) {

            int yPixel = topMargin + offsetY + (int)(lineYPositions[i] * renderScale);

            if (i == 0 || i == lineYPositions.length - 1) {

                g2d.setStroke(new BasicStroke(3));
            } else {

                float[] dash = {15.0f};
                g2d.setStroke(new BasicStroke(2, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10.0f, dash, 0.0f));
            }


            g2d.drawLine(0, yPixel, screenWidth, yPixel);
        }


        for (Vehicle v : vehicles) {

            int px = (int) (v.x * renderScale) + offsetX;
            int py = (int) (v.y * renderScale) + topMargin + offsetY;

            int carPixelLength = Math.max(12, (int) (v.LENGTH * renderScale));
            int carPixelWidth = Math.max(6, (int) (v.WIDTH * renderScale));


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
            String vehicleIdLabel = String.valueOf(v.id);
            FontMetrics vehicleIdMetrics = g2d.getFontMetrics();
            int vehicleIdTextWidth = vehicleIdMetrics.stringWidth(vehicleIdLabel);
            int vehicleIdTextY = (vehicleIdMetrics.getAscent() - vehicleIdMetrics.getDescent()) / 2;
            g2d.drawString(vehicleIdLabel, -vehicleIdTextWidth / 2, vehicleIdTextY);

            g2d.setTransform(oldTransform);


            g2d.setColor(Color.YELLOW);
            g2d.drawString(String.format("v:%.1f", v.speed), px - 15, py - 20);
            g2d.drawString(String.format("x:%.1f", v.x), px - 15, py - 6);
            if (renderDebugEnabled) {
                g2d.drawString(String.format("a:%.2f", v.plannedAcceleration), px - 15, py + 8);
                g2d.drawString(String.format("T:%.1f p:%.2f %s", v.targetSpeed, v.politeness, idmCooldownLabel(v)), px - 15, py + 22);
            }
            if (renderDebugEnabled && v.mobiling && !v.mobilDebug.isEmpty()) {
                int line = 0;
                List<Integer> mobilLanes = new ArrayList<>(v.mobilDebug.keySet());
                Collections.sort(mobilLanes);
                for (Integer mobilLane : mobilLanes) {
                    Vehicle.MobilDebugInfo info = v.mobilDebug.get(mobilLane);
                    g2d.drawString(String.format(
                            "to L%d F:%s R:%s self:%.2f karma:%.2f",
                            info.targetLane,
                            info.targetFrontVehicleId,
                            info.targetRearVehicleId,
                            info.selfBenefit,
                            info.karma
                    ), px - 15, py + 34 + line * 32);
                    g2d.drawString(String.format(
                            "oldR:%s ben:%.2f total:%.2f",
                            info.originalRearVehicleId,
                            info.originalRearBenefit,
                            info.overallBenefit
                    ), px - 15, py + 50 + line * 32);
                    line++;
                }
            }
        }
    }

    private String idmCooldownLabel(Vehicle vehicle) {
        if (vehicle instanceof IDMCooldownVehicle idmCooldownVehicle) {
            return String.format("idm:%.2f/%.2f",
                    idmCooldownVehicle.idmCooldownTimer,
                    idmCooldownVehicle.idmActionStepLength);
        }
        return "idm:-";
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
                    if (continueAfterNpcCollision && !isEgoInvolved(v1, v2)) {
                        continue;
                    }

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

    private boolean isEgoInvolved(Vehicle v1, Vehicle v2) {
        return isEgoVehicle(v1) || isEgoVehicle(v2);
    }

    private boolean isEgoVehicle(Vehicle vehicle) {
        return vehicle instanceof ControlledVehicle || "EGO".equals(vehicle.role);
    }

    public StarkShieldApp createStarkShieldApp(int futureSeconds) {
        if(this.vehicles == null || this.vehicles.isEmpty()) {
            throw new IllegalStateException("Engine must have vehicles to create StarkShieldApp");
        }

        return new StarkShieldApp(this, this.vehicles, futureSeconds);
    }

    public StarkShieldApp createStarkShieldApp(int futureSeconds, double shieldEgoRangeMeters,
                                               boolean randomizeHiddenTargetAndCooldown) {
        return createStarkShieldApp(futureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, false);
    }

    public StarkShieldApp createStarkShieldApp(int futureSeconds, double shieldEgoRangeMeters,
                                               boolean randomizeHiddenTargetAndCooldown,
                                               boolean checkChangeLaneToRearVehicleThreat) {
        return createStarkShieldApp(futureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                StarkShieldApp.DEFAULT_READ_SHIELD_IDM_COOLDOWN_TIMER);
    }

    public StarkShieldApp createStarkShieldApp(int futureSeconds, double shieldEgoRangeMeters,
                                               boolean randomizeHiddenTargetAndCooldown,
                                               boolean checkChangeLaneToRearVehicleThreat,
                                               boolean readShieldIdmCooldownTimer) {
        return createStarkShieldApp(futureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, StarkShieldApp.DEFAULT_FIX_PREDICTION,
                StarkShieldApp.DEFAULT_AGGRESSIVE_FINAL_STABILITY);
    }

    public StarkShieldApp createStarkShieldApp(int futureSeconds, double shieldEgoRangeMeters,
                                               boolean randomizeHiddenTargetAndCooldown,
                                               boolean checkChangeLaneToRearVehicleThreat,
                                               boolean readShieldIdmCooldownTimer,
                                               boolean fixPrediction) {
        return createStarkShieldApp(futureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, fixPrediction,
                StarkShieldApp.DEFAULT_AGGRESSIVE_FINAL_STABILITY);
    }

    public StarkShieldApp createStarkShieldApp(int futureSeconds, double shieldEgoRangeMeters,
                                               boolean randomizeHiddenTargetAndCooldown,
                                               boolean checkChangeLaneToRearVehicleThreat,
                                               boolean readShieldIdmCooldownTimer,
                                               boolean fixPrediction,
                                               boolean aggressiveFinalStability) {
        if(this.vehicles == null || this.vehicles.isEmpty()) {
            throw new IllegalStateException("Engine must have vehicles to create StarkShieldApp");
        }

        return new StarkShieldApp(this, this.vehicles, futureSeconds,
                shieldEgoRangeMeters, randomizeHiddenTargetAndCooldown,
                checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer,
                fixPrediction,
                aggressiveFinalStability,
                StarkShieldApp.DEFAULT_FINAL_STABILITY_PENALTY_MODE);
    }

    public StarkShieldApp createStarkShieldApp(int futureSeconds, double shieldEgoRangeMeters,
                                               boolean randomizeHiddenTargetAndCooldown,
                                               boolean checkChangeLaneToRearVehicleThreat,
                                               boolean readShieldIdmCooldownTimer,
                                               boolean fixPrediction,
                                               StarkShieldApp.FinalStabilityPenaltyMode finalStabilityPenaltyMode) {
        return createStarkShieldApp(futureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, fixPrediction,
                StarkShieldApp.DEFAULT_AGGRESSIVE_FINAL_STABILITY,
                finalStabilityPenaltyMode);
    }

    public StarkShieldApp createStarkShieldApp(int futureSeconds, double shieldEgoRangeMeters,
                                               boolean randomizeHiddenTargetAndCooldown,
                                               boolean checkChangeLaneToRearVehicleThreat,
                                               boolean readShieldIdmCooldownTimer,
                                               boolean fixPrediction,
                                               boolean aggressiveFinalStability,
                                               StarkShieldApp.FinalStabilityPenaltyMode finalStabilityPenaltyMode) {
        return createStarkShieldApp(futureSeconds, shieldEgoRangeMeters,
                randomizeHiddenTargetAndCooldown, checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer, fixPrediction, aggressiveFinalStability,
                finalStabilityPenaltyMode, null);
    }

    public StarkShieldApp createStarkShieldApp(int futureSeconds, double shieldEgoRangeMeters,
                                               boolean randomizeHiddenTargetAndCooldown,
                                               boolean checkChangeLaneToRearVehicleThreat,
                                               boolean readShieldIdmCooldownTimer,
                                               boolean fixPrediction,
                                               boolean aggressiveFinalStability,
                                               StarkShieldApp.FinalStabilityPenaltyMode finalStabilityPenaltyMode,
                                               Long hiddenStateRandomSeed) {
        if(this.vehicles == null || this.vehicles.isEmpty()) {
            throw new IllegalStateException("Engine must have vehicles to create StarkShieldApp");
        }

        return new StarkShieldApp(this, this.vehicles, futureSeconds,
                shieldEgoRangeMeters, randomizeHiddenTargetAndCooldown,
                checkChangeLaneToRearVehicleThreat,
                readShieldIdmCooldownTimer,
                fixPrediction,
                aggressiveFinalStability,
                finalStabilityPenaltyMode,
                hiddenStateRandomSeed);
    }


}
