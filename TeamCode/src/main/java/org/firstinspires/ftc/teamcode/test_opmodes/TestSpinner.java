package org.firstinspires.ftc.teamcode.test_opmodes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.cv.CVModule;
import org.firstinspires.ftc.teamcode.robot_components.input.Btn;
import org.firstinspires.ftc.teamcode.robot_components.input.Controller;
import org.firstinspires.ftc.teamcode.robot_components.robot.DuckSpinner;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

@RequiresApi(api = Build.VERSION_CODES.N)
@TeleOp(name="TestSpinner", group="Linear Opmode")
public class TestSpinner extends LinearOpMode implements FieldPositions {

    // Declare OpMode members
//    Robot robot;
    Controller controller1;
//    Controller controller2;
    DuckSpinner spinner;

    @Override
    public void runOpMode() {

        // Code to run ONCE when the driver hits INIT
//        robot = new Robot(hardwareMap, telemetry); // new CVModule(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1); // Whoever presses start + a
        DcMotor duckMotor = hardwareMap.get(DcMotor.class, "duckMotor");
        // TODO: what coefficient and radius should this be?
        spinner = new DuckSpinner(duckMotor, 0.5, .1, telemetry);

//        robot;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//        robot.cameras.webcam.pauseViewport();
//        robot.activateFieldLocalization();
//        telemetry.setMsTransmissionInterval(20);

//        robot.resetGyroAngle();
//        robot.resetElapsedTime();

//        ElapsedTime et = new ElapsedTime();

//        boolean started = false;
//
//        et.reset();

//        spinner.startRunning();
        while (opModeIsActive()) {

            // Registers controller input
            controller1.update();
//            controller2.update();

            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 1 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // NOTE: TO USE THESE FUNCTIONS PRESS START A
            //DRIVER FUNCTIONS
            if (controller1.right_bumper != Btn.PRESSED) {
//                robot.calculateDrivePowers(
//                        controller1.left_stick_x,
//                        controller1.left_stick_y,
//                        controller1.right_stick_x,
//                        controller1.right_stick_y
//                );
            }

//            if (et.time() > 1.0 && !started) {
//                spinner.startRunning();
//                started = true;
//            }

            if (controller1.b == Btn.PRESSED) {
                spinner.startRunning();
            }
            if (controller1.x == Btn.PRESSED) {
                spinner.stopRunning();
            }

            spinner.update();
            telemetry.update();

            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 2 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // NOTE: TO USE THESE FUNCTIONS, PRESS START B
            //OPERATOR FUNCTIONS

        }
    }
}
