package org.firstinspires.ftc.teamcode.robot_components.input;

import com.qualcomm.robotcore.hardware.Gamepad;

// Our custom controller class that gives us more control (no pun intended)
public class Controller extends Gamepad{
    Gamepad gamepad;

    public Btn a;
    public Btn b;
    public Btn x;
    public Btn y;
    public Btn dpad_right;
    public Btn dpad_up;
    public Btn dpad_left;
    public Btn dpad_down;
    public Btn back;
    public Btn guide;
    public Btn start;
    public Btn left_stick_button;
    public Btn right_stick_button;
    public Btn left_bumper;
    public Btn right_bumper;
    public Btn left_trigger_state;
    public Btn right_trigger_state;

    public double left_stick_x;
    public double left_stick_y;
    public double right_stick_x;
    public double right_stick_y;
    public double left_trigger;
    public double right_trigger;

    public static final double TRIGGER_PRESSED = 1;
    public static final double TRIGGER_RELEASED = 0;

    // Use for each controller in TeleOp
    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
        /*
        a = Btn.RELEASED;
        b = Btn.RELEASED;
        x = Btn.RELEASED;
        y = Btn.RELEASED;*/
        dpad_right = Btn.RELEASED;
        dpad_up = Btn.RELEASED;
        dpad_left = Btn.RELEASED;
        dpad_down = Btn.RELEASED;
        back = Btn.RELEASED;
        guide = Btn.RELEASED;
        start = Btn.RELEASED;
        left_stick_button = Btn.RELEASED;
        right_stick_button = Btn.RELEASED;
        left_bumper = Btn.RELEASED;
        right_bumper = Btn.RELEASED;
        left_trigger_state = Btn.RELEASED; // pretending the triggers are buttons (with >0.95 counting as being pressed)
        right_trigger_state = Btn.RELEASED;

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
    private Btn check(Btn previous, Boolean current) {
        Btn state;
        if (current) {
            if (previous == (Btn.RELEASED)) {
                state = Btn.PRESSING; // The button is about to be pressed down
            } else {
                state = Btn.PRESSED; // The button is held down
            }
        } else {
            if (previous == (Btn.PRESSED)) {
                state = Btn.RELEASING; // The button is about to be released
            } else {
                state = Btn.RELEASED; // The button is not being pushed
            }
        }
        return state;
    }

    // Returns true if any of the joysticks are being applied
    public boolean joyStickApplied() {
        return (right_stick_x != 0 || right_stick_y != 0 || left_stick_x != 0 || left_stick_y != 0);
    }
}
