package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot_components.cv.CVModule;
import org.firstinspires.ftc.teamcode.robot_components.input.Btn;
import org.firstinspires.ftc.teamcode.robot_components.input.Controller;

// Contains the basic code for a mecanum wheel drive base; should be extended by a child Robot class
@TeleOp(name="Tele1", group="Linear Opmode")
public class MotorTestControlHub extends LinearOpMode {

    // Motor powers
    protected double rightRearPower = 0;

    // Drive speed ranges from 0 to 1
    public double speed = 1;

    // For meta-drive
    // Specifies the direction of meta mode
    // 90 degrees is optimal for when the driver is standing on side of field

    // Mecanum wheel drive motors
    public DcMotor motor;

    // For displaying things on the DS phone
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    // Declare OpMode members
    CVModule robot;
    Controller controller1;

    @Override
    public void runOpMode() {

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
                motor.setPower(.5);
            }


        }
    }
}
