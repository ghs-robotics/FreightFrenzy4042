package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot_components.navigation.Gyro;
import org.firstinspires.ftc.teamcode.robot_components.navigation.OdometryModule;

@TeleOp(name="EncoderTest", group="Iterative Opmode")
public class EncoderTestTele extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
//    private DcMotor leftRearDrive = null;
//    private DcMotor rightRearDrive = null;
    private Gyro gyro;
    private OdometryModule odo;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftFrontDrive = hardwareMap.get(DcMotor.class, "odo");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motor2");
//        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
//        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        gyro = new Gyro(hardwareMap);
        odo = new OdometryModule(leftFrontDrive, rightFrontDrive, gyro);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
//        leftDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        int encoderValues[] = odo.getRawEncoderValues();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("encoder values", "x %d right %d", encoderValues[0], encoderValues[1]);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
