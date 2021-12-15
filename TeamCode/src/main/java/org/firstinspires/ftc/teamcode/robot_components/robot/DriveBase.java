package org.firstinspires.ftc.teamcode.robot_components.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot_components.movement_enhancing.PIDController;
import org.firstinspires.ftc.teamcode.robot_components.navigation.Gyro;

// Contains the basic code for a mecanum wheel drive base; should be extended by a child Robot class
public class DriveBase {

    // Motor powers
    protected double leftFrontPower = 0;
    protected double rightFrontPower = 0;
    protected double leftRearPower = 0;
    protected double rightRearPower = 0;

    protected double batteryVoltage;

    // Drive speed ranges from 0 to 1
    public double speed = 1.0;
    public double autoSpeed = 0.8;


    // Mecanum wheel drive motors
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftRearDrive;
    public DcMotor rightRearDrive;

    public Gyro gyro; // Keeps track of robot's angle
    public PIDController gyroPID; // Controls the angle for automated functions
    public double targetGyroAngle = 0; // gyroscope will target this angle

    // For displaying things on the DS phone
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    public ElapsedTime elapsedTime;

    // Constructs a DriveBase object with four drive motors
    public DriveBase(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwareMap = hardwareMap;

        // These are the names to use in the phone config (in quotes below)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");

        // Default is to have the launcher be the front
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        // Makes the drive motors apply resistance when the power is zero
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initializes gyro and sets starting angle to zero
        gyro = new Gyro(hardwareMap);
        resetGyroAngle();

        // gyroPID works best when Ki = 0
        gyroPID = new PIDController(0.0330, 0.0000, 0.0020, 0.2);

        // Initializes telemetry
        this.telemetry = telemetry;

        // For keeping track of time
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        batteryVoltage = getBatteryVoltage();
    }

    // Displays drive motor powers on the DS phone
    public void addTelemetryData() {
        telemetry.addData("LF power: ", leftFrontPower);
        telemetry.addData("RF power: ", rightFrontPower);
        telemetry.addData("LR power: ", leftRearPower);
        telemetry.addData("RR power: ", rightRearPower);
        telemetry.update();
    }

    // Returns true if any of the drive motors are running
    public boolean driveMotorsRunning() {
        return (leftFrontPower != 0 || rightFrontPower != 0 || leftRearPower != 0 || rightRearPower != 0);
    }

    // Calculates powers for mecanum wheel drive
    public void calculateDrivePowers(double x, double y, double r, boolean isAuto) {
        r = -r;

        double multiplier = isAuto ? autoSpeed : speed; // use autospeed if isAuto else use speed
    // set motor powers, assumed that positive power = forwards motion for wheel, there's often a motor.reverse() function to help with this
        rightFrontPower = multiplier * (y - x + r);
        leftFrontPower = multiplier * (y + x - r);
        leftRearPower = multiplier * (y - x - r);
        rightRearPower = multiplier * (y + x + r);
    }

    public void calculateDrivePowersOffset(double x, double y, double r, double offset, boolean isAuto) {
        offset = Math.toRadians(offset);
        x = x * Math.cos(offset) - y * Math.sin(offset);
        y = x * Math.sin(offset) + y * Math.cos(offset);

        double multiplier = isAuto ? autoSpeed : speed; // use autospeed if isAuto else use speed

        r = -r;
        // set motor powers, assumed that positive power = forwards motion for wheel, there's often a motor.reverse() function to help with this
        rightFrontPower = multiplier * (y - x + r);
        leftFrontPower = multiplier * (y + x - r);
        leftRearPower = multiplier * (y - x - r);
        rightRearPower = multiplier * (y + x + r);
        /*telemetry.addData("lf", leftFrontPower);
        telemetry.addData("lr", leftRearPower);
        telemetry.addData("rf", rightFrontPower);
        telemetry.addData("rr", rightRearPower);
        telemetry.update(); */
    }

    // Returns how many seconds have passed since the timer was last reset
    public double elapsedSecs() {
        return elapsedTime.seconds();
    }

    // Finds the current battery voltage
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) { result = Math.min(result, voltage);
            }
        }
        return result;
    }

    // Finds an equivalent gyro angle (mod 360) within range of the actual current robot angle
    // Useful if you don't want your robot to spin in circles a bunch of times when calling rotateToPos
    public double getReasonableGyroAngle(double desiredAngle) {
        double actualAngle = gyro.getAngle();
        while (Math.abs(actualAngle - desiredAngle)  > 190) {
            if (desiredAngle < actualAngle) {
                desiredAngle += 360;
            } else {
                desiredAngle -= 360;
            }
        }
        return desiredAngle;
    }

    public double getAbsoluteGyroError() {
        return Math.abs(getGyroError());
    }

    public double getGyroError() {
        return targetGyroAngle - gyro.getAngle();
    }

    public double getGyroPIDValue() {
        return -gyroPID.calcVal(getGyroError());
    }

    // Resets the timer
    public void resetElapsedTime() {
        elapsedTime.reset();
    }

    // Sets current angle as zero
    public void resetGyroAngle() {
        gyro.resetAngle();
    }

    // Sends power to drive motors
    public void sendDrivePowers() {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
    }

    // Sets speed to desired value
    public void setDriveSpeed(double speed) {
        this.speed = speed;
    }

    // Update the targetGyroAngle (before calling, say, an automated function that uses rotation)
    public void setTargetGyroAngle(double angle) {
        targetGyroAngle = getReasonableGyroAngle(angle);
    }

    // Makes the robot stop driving
    public void stopDrive() {
        calculateDrivePowersOffset(0, 0, 0, 0, false);
        sendDrivePowers();
    }

    // Toggles the drive speed between 60% and normal
    public void toggleSpeed() {
        speed = (speed == 1 ? 0.6 : 1);
    }

    // Displays telemetry values and updates the powers being sent to the drive motors
    public void updateDrive() {
        sendDrivePowers();
        addTelemetryData();
    }

    // Makes the robot wait (i.e. do nothing) for a specified number of seconds
    public void wait(double seconds) {
        double start = elapsedSecs();
        while (elapsedSecs() - start < seconds) {}
    }
}
