package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.util.ElapsedTime;

// For use in TeleOp; allows you to toggle between smooth and choppy driving
// NOTE: For Ultimate Goal, this wasn't particularly useful, so we didn't use it
public class SmoothnessRegulator {

    public static boolean smooth = false; // Whether the drive mode should be smooth or choppy
    public static boolean running = false; // True if the joysticks indicate that the robot should be moving

    public static double factor = 1.0; // Factor to multiply the drive powers by

    private static ElapsedTime elapsedTime = new ElapsedTime(); // To keep track of time
    private static Controller c; // Controller for joystick input

    // Smooth mode
    public static void activateSmoothMode() {
        smooth = true;
    }

    // Choppy mode
    public static void deactivateSmoothMode() {
        smooth = false;
    }

    // Specifies the controller to take input from
    public static void setController(Controller controller) {
        c = controller;
    }

    // Toggles between smooth (slower acceleration) and choppy (accelerates to full speed immediately) drive mode
    public static void switchMode() {
        smooth = !smooth;
    }

    // Returns factor to multiply motor powers by
    public static double getFactor() {
        if (!smooth) {
            return 1.0;
        }
        return factor;
    }

    // Updates the factor as well as the status (running vs not running)
    public static void update() {
        if (c.left_stick_x != 0 || c.left_stick_y != 0 || c.right_stick_x != 0) { // Any drive motor running
            if (!running) {
                running = true;
                elapsedTime.reset();
            } else {
                factor = Math.min(1.0, 0.3 + 0.7 * elapsedTime.seconds());
            }
        } else { // No drive motors running
            running = false;
        }
    }
}
