package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.input.Btn;
import org.firstinspires.ftc.teamcode.robot_components.input.Controller;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

import java.util.Objects;

@TeleOp(name="MotorFix", group="Linear Opmode")
public class MotorFix extends LinearOpMode implements FieldPositions {

    // Declare OpMode members
    Robot robot;
    Controller controller1;
    Controller controller2;

    @Override
    public void runOpMode() {

        // Code to run ONCE when the driver hits INIT
        robot = new Robot(hardwareMap, telemetry); // new CVModule(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1); // Whoever presses start + a
        controller2 = new Controller(gamepad2); // Whoever presses start + b

//        robot;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//        robot.cameras.webcam.pauseViewport();
//        robot.activateFieldLocalization();
//        telemetry.setMsTransmissionInterval(20);

        robot.resetGyroAngle();
        robot.resetElapsedTime();

        while (opModeIsActive()) {

            // Registers controller input
            controller1.update();
            controller2.update();

            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 1 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // NOTE: TO USE THESE FUNCTIONS PRESS START A
            //DRIVER FUNCTIONS
            robot.calculateDrivePowers(
                    controller1.left_stick_x,
                    controller1.left_stick_y,
                    controller1.right_stick_x
            );
            robot.sendDrivePowers();

            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 2 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // NOTE: TO USE THESE FUNCTIONS, PRESS START B
            //OPERATOR FUNCTIONS

            //toggles dropper, make code that goes down and then back up later
            if (controller2.a == Btn.PRESSED) {
                double startTime = robot.elapsedSecs();
                robot.startMotor(0);
                while(startTime + 5 < robot.elapsedSecs()) {}
                robot.stopMotors();
            }

            if (controller2.b == Btn.PRESSED) {
                double startTime = robot.elapsedSecs();
                robot.startMotor(1);
                while(startTime + 5 < robot.elapsedSecs()) {}
                robot.stopMotors();
            }

            if (controller2.x == Btn.PRESSED) {
                double startTime = robot.elapsedSecs();
                robot.startMotor(2);
                while(startTime + 5 < robot.elapsedSecs()) {}
                robot.stopMotors();
            }

            if (controller2.y == Btn.PRESSED) {
                double startTime = robot.elapsedSecs();
                robot.startMotor(3);
                while(startTime + 5 < robot.elapsedSecs()) {}
                robot.stopMotors();
            }

            telemetry.update();
        }
    }
}