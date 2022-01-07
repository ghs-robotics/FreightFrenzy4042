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

    private final double INTAKE_POWER = 1;

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
                    -1 * controller1.left_stick_x,
                    controller1.left_stick_y,
                    -1 * controller1.right_stick_x,
                    false
            );

            robot.sendDrivePowers();

            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 2 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // NOTE: TO USE THESE FUNCTIONS, PRESS START B
            //OPERATOR FUNCTIONS
            /*ADD CODE: FINE CONTROL ARM VIA RIGHT JOYSTICK Y, DROPPER VIA L/R BUMPER
            * RESET DROPPER TO NEUTRAL, forwards, backwards ON R JOYSTICK PRESSED*/
            /*
            * Y: extend to high arm position, A: extend to low arm position
            * B: duckspinner
            * DPAD UP: autorun front intake, DPAD DOWN: autorun back intake
            * left stick y: both intakes forwards/backwards, hold left stick for slow mode
            * right stick y: change dropper position, up = back, dwn = fwds
            * L/R BUMPERS: set servo pos*/
            // make sure you make the target negative
            boolean extenderGotoHigh = controller2.y == Btn.PRESSING;
            boolean extenderGotoLow = controller2.a == Btn.PRESSING;

            if(extenderGotoHigh) {
                //im gonna hardcode the distance because heck you - simon
                if(shouldGoToHigh) {
                    robot.moveEntenderTo((int)Robot.EXT_OUT);
                    robot.neutralDropperPosition();
                } else {
                    robot.moveEntenderTo((int)Robot.EXT_IN);
                }

                shouldGoToHigh = !shouldGoToHigh;
            }

            double duckFront = controller2.left_trigger;
            double duckBack = controller2.right_trigger;

            if (duckFront > 0) {
                robot.spinRedDirection(1);
            } else if (duckBack > 0) {
                robot.spinBlueDirection(1);
            } else {
                robot.spinRedDirection(0); //This could be red or blue and it would still work
            }

            if(extenderGotoLow) {
                if(shouldGoToLow) {
                    robot.moveEntenderTo((int)Robot.EXT_LOW);
                    robot.neutralDropperPosition();
                } else {
                    robot.moveEntenderTo((int)Robot.EXT_IN);
                }
                shouldGoToLow = !shouldGoToLow;
            }

            boolean TOGGLE_INTAKE_SPEED = controller2.left_stick_button == Btn.PRESSING;
            if(TOGGLE_INTAKE_SPEED) {
                robot.setBackIntakePower(-1 * controller2.left_stick_x*0.2);
                robot.setFrontIntakePower(controller2.left_stick_x*0.2);
            }else{
                robot.setFrontIntakePower(-1 * controller2.left_stick_x);
                robot.setBackIntakePower(controller2.left_stick_x);
            }

            telemetry.addData("arm encoder", robot.extenderMotor.getCurrentPosition()+"");

            boolean drop = controller2.left_stick_button == Btn.PRESSED;
            if(drop){
                robot.dropperServo.setPosition(((controller2.left_stick_x)+1)/2);
            }else {
                robot.dropperServo.setPosition(Math.max(Math.min(((controller2.left_stick_x) + 1) / 2,
                        Robot.DROPPER_BACK), Robot.DROPPER_FORWARD));
            }
            telemetry.addData( "right stick pressed", controller2.left_stick_button == Btn.PRESSED);

            boolean dropBack = controller2.left_bumper == Btn.PRESSING;
            if(dropBack){
                robot.backDropperPosition();
            }
            boolean dropForward = controller2.right_bumper == Btn.PRESSING;
            if(dropForward){
                robot.forwardDropperPosition();
            }

            //there is a delay between when you press the button and the servo starts spinning
            //duck spinner
            /*boolean SPINNER = controller2.b == Btn.PRESSED;*/
            //robot.spinnerServo.setPower(controller2.right_trigger);

            telemetry.addData("right trigger", controller2.right_trigger);
            telemetry.addData("spinner power", robot.spinnerServoRed.getPower());
            telemetry.addData("dropper servo pos", robot.dropperServo.getPosition());

            telemetry.update();
        }
    }
}
