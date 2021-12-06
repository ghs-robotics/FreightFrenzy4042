package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot_components.input.Btn;
import org.firstinspires.ftc.teamcode.robot_components.input.Controller;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

@TeleOp(name="MainTele", group="Linear Opmode")
public class MainTele extends LinearOpMode{
    
    // Declare OpMode members
    Robot robot;
    Controller controller1;
    Controller controller2;

    private boolean shouldGoToHigh = true;
    private boolean shouldGoToLow = true;


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
            // NOTE: NEED TO REFINE CONTROLS WITH DRIVE TEAM
            // DRIVER FUNCTIONS

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
//extends the arm to the high goal
            if(controller2.y == Btn.PRESSING) {
                //im gonna hardcode the distance because heck you - simon
                if(shouldGoToHigh)
                    robot.drive(-2000);
                else
                    robot.drive(-10);
                shouldGoToHigh = !shouldGoToHigh;
            }

            //extends the arm to the low goal
            if(controller2.a == Btn.PRESSING) {
                if(shouldGoToLow)
                    robot.drive(-1000);
                else
                    robot.drive(-10);
                shouldGoToLow = !shouldGoToLow;
            }
            //intake
            //run intake based on how strong the right trigger is pressed
                robot.setIntakePower(0.9 * controller2.right_stick_y);

                //robot.setExtenderPower(-controller2.left_stick_x);

                telemetry.addData("arm encoder", robot.extenderMotor.getCurrentPosition()+"");
            //turn bucket up/down

            //there is a delay between when you press the button and the servo starts spinning
            //moved duck spinner code here because the y button seems it will be used for something else
            if(gamepad2.b) {
                robot.spinnerServo.setPower(1);
            } else
                robot.spinnerServo.setPower(0);


            if (controller2.dpad_up == Btn.PRESSING) {
                robot.dropperServo.setPosition(robot.dropperServo.getPosition() + 0.05);
            } else if (controller2.dpad_down == Btn.PRESSING) {
                robot.dropperServo.setPosition(robot.dropperServo.getPosition() - 0.05);
            }
            telemetry.addData("dropper servo pos", robot.dropperServo.getPosition());

            telemetry.update();
        }
    }
}
