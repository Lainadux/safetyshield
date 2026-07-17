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

import RefractoredVersion.TestScript.Config.FallBackMode;
import RefractoredVersion.TestScript.Config.JavaMomentumConfig;
import lombok.Getter;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Cursor;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.KeyboardFocusManager;
import java.awt.Point;
import java.awt.RenderingHints;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.AffineTransform;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
@Getter
public class JavaHighwayEngine {
    private int frequency = 20;
    private double dt = (double) 1 / frequency;
    public void setFrequency(int frequency) {
        this.frequency = frequency;
        this.dt = (double) 1 / frequency;
    }

    public boolean isDecisionTime(){
        return stepsTaken % frequency == 0;
    }
    public FallBackMode fallBackMode = FallBackMode.DEFAULT;
    public JavaMomentumConfig config;
    public List<Vehicle> vehicles;
    public int numLanes = 3;
    public double timeElapsed = 0;
    public int stepsTaken =0;
    private JFrame frame;
    private JPanel renderPanel;
    private static final double DEFAULT_RENDER_SCALE = 7.0;
    private static final double MIN_RENDER_SCALE = 3.0;
    private static final double MAX_RENDER_SCALE = 25.0;
    private double renderScale = DEFAULT_RENDER_SCALE;
    private int cameraDragOffsetX = 0;
    private int cameraDragOffsetY = 0;
    private Point lastCameraDragPoint;
    private volatile boolean spaceHeld = false;
    private volatile boolean paused = false;
    private boolean controlsInstalled = false;
    private int lastPausedDecisionStep = -1;

    public void checkCollision(){
        for (Vehicle vehicle : vehicles) {
            try {
                vehicle.checkCollision();
            } catch (RuntimeException e) {
                paused = true;
                throw e;
            }
        }
    }

    public void planActions() throws Exception {
        for(Vehicle v: vehicles){
            if(v instanceof NonNpcVehicle){
                NonNpcVehicle nonNpcVehicle = (NonNpcVehicle) v;
                nonNpcVehicle.planAction();
            }
            else {
                v.planAction(vehicles);
            }


        }
    }

    public void applyPhysics(){
        for(Vehicle v: vehicles){
            v.applyPhysics();
        }

    }
    public void step() throws Exception {
        if (paused) {
            return;
        }
        syncEngineReferences();
        planActions();
        if (isDecisionTime()) {
            updatePreviousDecisionSpeeds();
        }
        if (isDecisionTime()) {
            renderCurrentFrame();
        }
        waitAtDecisionTimeUnlessSpaceHeld();
        applyPhysics();
        checkCollision();
        timeElapsed += dt;
        stepsTaken += 1;

    }

    private void updatePreviousDecisionSpeeds() {
        for (Vehicle vehicle : vehicles) {
            vehicle.previousSecondSpeed = vehicle.speed;
        }
    }

    public void render() {
        syncEngineReferences();
        renderCurrentFrame();

        try {
            Thread.sleep((long) (dt * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void renderCurrentFrame() {
        if (frame == null) {
            initUI();
        }

        renderPanel.repaint();
    }

    private void initUI() {
        frame = new JFrame("Java Highway Simulator");
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
        setupCameraControls();
        setupKeyboardControls();

        frame.add(renderPanel, BorderLayout.CENTER);
        frame.setVisible(true);
    }

    private void setupKeyboardControls() {
        if (controlsInstalled) {
            return;
        }
        controlsInstalled = true;
        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(event -> {
            if (event.getKeyCode() == KeyEvent.VK_SPACE) {
                if (event.getID() == KeyEvent.KEY_PRESSED) {
                    spaceHeld = true;
                } else if (event.getID() == KeyEvent.KEY_RELEASED) {
                    spaceHeld = false;
                }
            }
            return false;
        });
    }

    private void setupCameraControls() {
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

    private void waitAtDecisionTimeUnlessSpaceHeld() {
        if (!isDecisionTime() || spaceHeld || lastPausedDecisionStep == stepsTaken) {
            return;
        }
        lastPausedDecisionStep = stepsTaken;
        CountDownLatch spacePressed = new CountDownLatch(1);
        KeyboardFocusManager focusManager = KeyboardFocusManager.getCurrentKeyboardFocusManager();
        java.awt.KeyEventDispatcher dispatcher = event -> {
            if (event.getID() == KeyEvent.KEY_PRESSED && event.getKeyCode() == KeyEvent.VK_SPACE) {
                spaceHeld = true;
                spacePressed.countDown();
                return true;
            }
            return false;
        };

        focusManager.addKeyEventDispatcher(dispatcher);
        try {
            spacePressed.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            focusManager.removeKeyEventDispatcher(dispatcher);
        }
    }

    private void drawHighway(Graphics2D g2d) {
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        int screenWidth = renderPanel.getWidth();
        int topMargin = 100;
        double cameraX = vehicles == null || vehicles.isEmpty() ? 0 : getCameraVehicle().x;
        int offsetX = (int) (screenWidth / 2 - cameraX * renderScale) + cameraDragOffsetX;
        int offsetY = cameraDragOffsetY;

        g2d.setColor(Color.WHITE);
        for (int lane = 0; lane <= numLanes; lane++) {
            int yPixel = topMargin + offsetY + (int) ((lane * 4.0 - 2.0) * renderScale);
            if (lane == 0 || lane == numLanes) {
                g2d.setStroke(new BasicStroke(3));
            } else {
                float[] dash = {15.0f};
                g2d.setStroke(new BasicStroke(2, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10.0f, dash, 0.0f));
            }
            g2d.drawLine(0, yPixel, screenWidth, yPixel);
        }

        if (vehicles == null) {
            return;
        }

        for (Vehicle v : vehicles) {
            int px = (int) (v.x * renderScale) + offsetX;
            int py = (int) (v.y * renderScale) + topMargin + offsetY;
            int carPixelLength = Math.max(12, (int) (v.LENGTH * renderScale));
            int carPixelWidth = Math.max(6, (int) (v.WIDTH * renderScale));

            AffineTransform oldTransform = g2d.getTransform();
            g2d.translate(px, py);
            g2d.rotate(v.heading);
            g2d.setColor(v instanceof NonNpcVehicle ? new Color(0, 200, 255) : new Color(255, 80, 80));
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
        }

        g2d.setColor(Color.WHITE);
        g2d.drawString(String.format("t: %.2f  step: %d%s", timeElapsed, stepsTaken,
                isDecisionTime() ? "  decision" : ""), 12, 22);
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

    private void syncEngineReferences() {
        if (vehicles == null) {
            return;
        }
        for (Vehicle vehicle : vehicles) {
            vehicle.setEngine(this);
        }
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




}
