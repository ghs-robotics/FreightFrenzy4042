package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.Gamepad;

// Our custom controller class that gives us more control (no pun intended)
public class Controller {
    Gamepad gamepad;

    public String a;
    public String b;
    public String x;
    public String y;
    public String dpad_right;
    public String dpad_up;
    public String dpad_left;
    public String dpad_down;
    public String back;
    public String guide;
    public String start;
    public String left_stick_button;
    public String right_stick_button;
    public String left_bumper;
    public String right_bumper;
    public String left_trigger_state;
    public String right_trigger_state;

    public double left_stick_x;
    public double left_stick_y;
    public double right_stick_x;
    public double right_stick_y;
    public double left_trigger;
    public double right_trigger;

    // Use for each controller in TeleOp
    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;

        a = "released";
        b = "released";
        x = "released";
        y = "released";
        dpad_right = "released";
        dpad_up = "released";
        dpad_left = "released";
        dpad_down = "released";
        back = "released";
        guide = "released";
        start = "released";
        left_stick_button = "released";
        right_stick_button = "released";
        left_bumper = "released";
        right_bumper = "released";
        left_trigger_state = "released"; // pretending the triggers are buttons (with >0.95 counting as being pressed)
        right_trigger_state = "released";

        left_stick_x = 0;
        left_stick_y = 0;
        right_stick_x = 0;
        right_stick_y = 0;
        left_trigger = 0;
        right_trigger = 0;
    }

    // Update controller input
    public void update() {
        a = check(a, gamepad.a);
        b = check(b, gamepad.b);
        x = check(x, gamepad.x);
        y = check(y, gamepad.y);

        dpad_right = check(dpad_right, gamepad.dpad_right);
        dpad_up    = check(dpad_up,    gamepad.dpad_up);
        dpad_left  = check(dpad_left,  gamepad.dpad_left);
        dpad_down  = check(dpad_down,  gamepad.dpad_down);

        left_trigger_state  = check(left_trigger_state, gamepad.left_trigger > 0.95);
        right_trigger_state = check(right_trigger_state,    gamepad.right_trigger > 0.95);

        back  = check(back,  gamepad.back);
        guide = check(guide, gamepad.guide);
        start = check(start, gamepad.start);

        left_stick_button  = check(left_stick_button,  gamepad.left_stick_button);
        right_stick_button = check(right_stick_button, gamepad.right_stick_button);

        left_bumper  = check(left_bumper,  gamepad.left_bumper);
        right_bumper = check(right_bumper, gamepad.right_bumper);

        left_stick_x = gamepad.left_stick_x;
        left_stick_y = -gamepad.left_stick_y;

        right_stick_x = gamepad.right_stick_x;
        right_stick_y = -gamepad.right_stick_y;

        left_trigger  = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
    }

    // Helper method for updating controller input
    private String check(String previous, Boolean current) {
        String state;
        if (current) {
            if (previous.equals("released")) {
                state = "pressing"; // The button is about to be pressed down
            } else {
                state = "pressed"; // The button is held down
            }
        } else {
            if (previous.equals("pressed")) {
                state = "releasing"; // The button is about to be released
            } else {
                state = "released"; // The button is not being pushed
            }
        }
        return state;
    }

    // Returns true if any of the joysticks are being applied
    public boolean joyStickApplied() {
        return (right_stick_x != 0 || right_stick_y != 0 || left_stick_x != 0 || left_stick_y != 0);
    }
}
