package org.firstinspires.ftc.teamcode.teleop;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.input.Btn;
import org.firstinspires.ftc.teamcode.robot_components.input.Controller;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;
import org.firstinspires.ftc.teamcode.robot_components.robot.SimpleDuckSpinner;

@RequiresApi(api = Build.VERSION_CODES.N) // enable java 8
@TeleOp(name="TestTele", group="Linear Opmode")
public class TestTele extends LinearOpMode implements FieldPositions {
    
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
        SimpleDuckSpinner spinner = new SimpleDuckSpinner(robot.spinnerMotor, 0.0, telemetry, DcMotorSimple.Direction.FORWARD);

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

        double runtimeSeconds = 0.0;
        double runtimeSecondsLast = 0.0;

        while (opModeIsActive()) {
            runtimeSeconds = robot.elapsedTime.seconds();
            double deltaTime = runtimeSeconds - runtimeSecondsLast;
            runtimeSecondsLast = runtimeSeconds;


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
                    controller1.right_stick_x,
                    controller1.right_stick_y
            );
            robot.sendDrivePowers();

            if (controller1.x == Btn.PRESSED) {
                spinner.setDirection(DcMotorSimple.Direction.FORWARD);
                spinner.startRunning();
            }

            if (controller1.b == Btn.PRESSED) {
                spinner.setDirection(DcMotorSimple.Direction.REVERSE);
                spinner.startRunning();
            }

            if (controller1.y == Btn.PRESSED) {
                spinner.stopRunning();
            }

            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 2 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // NOTE: TO USE THESE FUNCTIONS, PRESS START B
            //OPERATOR FUNCTIONS

            //toggles dropper, make code that goes down and then back up later
            if (controller2.a == Btn.PRESSING) {
                robot.dropGameElement();
            }

            //intake
            //run intake based on how strong trigger pressed (i think)
            boolean triggerPressed = Math.abs(controller2.right_trigger - Controller.TRIGGER_PRESSED) < .01;
            if (triggerPressed) {
                robot.setIntakePower(0.9 * controller2.right_trigger);
            }

            //turn bucket up/down
            if(controller2.b == Btn.PRESSING) {
                robot.toggleIntakeServo();
            }

            //extend the arm
            if(controller2.x == Btn.PRESSING) {
                //im gonna hardcode the distance because frick you - simon
                robot.toggleExtension(13);
            }

            if (controller2.left_stick_y > 0.1 || controller2.left_stick_y < -0.1) {
                robot.extenderMotor.setTargetPosition(
                        (int) Math.round (
                                robot.extenderMotor.getTargetPosition()
                                + controller2.left_stick_y * deltaTime
                        )
                );
            }

        }
    }
}
