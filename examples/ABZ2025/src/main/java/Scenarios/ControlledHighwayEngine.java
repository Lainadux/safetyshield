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

package Scenarios;

import java.awt.*;
import java.awt.event.KeyEvent;

public class ControlledHighwayEngine extends HighwayEngine {

    private volatile boolean isPaused = true;
    private volatile boolean stepRequested = false;

    public ControlledHighwayEngine(double dt) {
        super(dt);
        setupKeyListener();
        System.out.println("space for pause/unpause, N for step when paused");
    }

    private void setupKeyListener() {

        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(new KeyEventDispatcher() {
            @Override
            public boolean dispatchKeyEvent(KeyEvent e) {

                if (e.getID() == KeyEvent.KEY_PRESSED) {
                    if (e.getKeyCode() == KeyEvent.VK_SPACE) {
                        isPaused = !isPaused;
                        System.out.println(isPaused ? "paused" : "continue");
                    } else if (e.getKeyCode() == KeyEvent.VK_N) {

                        if (isPaused) {
                            stepRequested = true;
                        }
                    }
                }
                return false;
            }
        });
    }


    @Override
    public void step() throws Exception {

        if (isPaused && !stepRequested) {
            return;
        }


        if (stepRequested) {
            stepRequested = false;

        }


        super.step();
    }
}