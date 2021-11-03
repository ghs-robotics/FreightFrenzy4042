package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot_components.input.Btn;
import org.firstinspires.ftc.teamcode.robot_components.input.Controller;

// Contains the basic code for a mecanum wheel drive base; should be extended by a child Robot class
@TeleOp(name="motor test", group="Linear Opmode")
public class MotorTestControlHub extends LinearOpMode {

    // Motor powers
    protected double motor_power = 0;
    // Drive speed ranges from 0 to 1
    public double speed = 1;

    // For meta-drive
    // Specifies the direction of meta mode
    // 90 degrees is optimal for when the driver is standing on side of field

    // Mecanum wheel drive motors
    public DcMotor motor1;
    public DcMotor motor2;

    // Declare OpMode members
    Controller controller1;

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        // Code to run ONCE when the driver hits INIT
        controller1 = new Controller(gamepad1); // Whoever presses start + a


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            // Registers controller input
            controller1.update();

            if(controller1.a == Btn.PRESSED) {
                motor1.setPower(.5);
                motor2.setPower(.5);
            }

            if(controller1.b == Btn.PRESSED) {
                motor1.setPower(0);
                motor2.setPower(0);
            }

        }
    }
}
