package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot_components.input.Btn;
import org.firstinspires.ftc.teamcode.robot_components.input.Controller;
import org.firstinspires.ftc.teamcode.robot_components.navigation.Gyro;
import org.firstinspires.ftc.teamcode.robot_components.navigation.OdometryModule;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

@TeleOp (name = "TestTele", group = "Iterative Opmode")
public class TestTele extends OpMode {

    // Declare OpMode members
    Robot robot;
    Controller controller1;
    Controller controller2;

    private boolean shouldGoToHigh = true;
    private boolean shouldGoToLow = true;

    private boolean extended = true;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    //private DcMotor leftRearDrive = null;
    //private DcMotor rightRearDrive = null;
    private Gyro gyro;
    private OdometryModule odo;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftFrontDrive = hardwareMap.get(DcMotor.class, "odo");
//        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
//        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        gyro = new Gyro(hardwareMap);
        odo = new OdometryModule(leftFrontDrive, rightFrontDrive, gyro);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
//        leftDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.

        robot = new Robot(hardwareMap, telemetry); // new CVModule(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1); // Whoever presses start + a
        controller2 = new Controller(gamepad2); // Whoever presses start + b

//        robot;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        init();
//        robot.cameras.webcam.pauseViewport();
//        robot.activateFieldLocalization();
//        telemetry.setMsTransmissionInterval(20);

        robot.resetGyroAngle();
        robot.resetElapsedTime();
    }

    @Override
    public void init_loop(){
    }

    @Override
    public void start(){
    // Code to run ONCE when the driver hits INIT
        runtime.reset();

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

        // make sure you make the target negative
        boolean HIGH_EXTENDER = controller2.y == Btn.PRESSING;
        boolean LOW_EXTENDER = controller2.a == Btn.PRESSING;

        if(HIGH_EXTENDER) {
            //im gonna hardcode the distance because heck you - simon
            if(shouldGoToHigh) {
                robot.moveEntenderTo(-3700);
                robot.neutralDropperPosition();
            } else {
                robot.moveEntenderTo(-10);
            }

            shouldGoToHigh = !shouldGoToHigh;
        }

        //extends the arm to the low goal
        if(LOW_EXTENDER) {
            if(shouldGoToLow) {
                robot.moveEntenderTo(-1500);
                robot.neutralDropperPosition();
            } else {
                robot.moveEntenderTo(-10);
            }
            shouldGoToLow = !shouldGoToLow;
        }

        boolean FRONT_INTAKE = controller2.dpad_up == Btn.PRESSED;
        boolean BACK_INTAKE = controller2.dpad_down == Btn.PRESSED;

        //front intake
        //run intake based on how strong the right trigger is pressed
        if(FRONT_INTAKE) {
            robot.setFrontIntakePower(0.9);
            robot.forwardDropperPosition();
        } else {
            robot.setFrontIntakePower(0);
        }

        //back intake
        if(BACK_INTAKE) {
            robot.setBackIntakePower(0.9);
            robot.backDropperPosition();
        } else {
            robot.setBackIntakePower(0);
        }

        telemetry.addData("extender encoder", robot.extenderMotor.getCurrentPosition()+"");
        //turn bucket up/down

        //there is a delay between when you press the button and the servo starts spinning
        //duck spinner
        boolean SPINNER = controller2.b == Btn.PRESSED;

        if(SPINNER) {
            robot.spinnerServo.setPower(1);
        } else {
            robot.spinnerServo.setPower(0);
        }
        telemetry.addData("dropper servo pos", robot.dropperServo.getPosition());

        telemetry.update();
    }

    @Override
    public void loop() {
        int encoderValues[] = odo.getRawEncoderValues();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("encoder values", "x %d right %d", encoderValues[0], encoderValues[1]);
    }

    @Override
    public void stop() {
    }
}
