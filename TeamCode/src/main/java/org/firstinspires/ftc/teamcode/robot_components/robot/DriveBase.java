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
    public double speed = 1;

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

    // Turns the robot to a desired target angle (if called repeatedly)
    public void adjustAngle() {
        calculateDrivePowers(0, 0, getGyroPIDValue());
        sendDrivePowers();
    }

    // Returns true if any of the drive motors are running
    public boolean driveMotorsRunning() {
        return (leftFrontPower != 0 || rightFrontPower != 0 || leftRearPower != 0 || rightRearPower != 0);
    }

    // Calculates powers for mecanum wheel drive

    //much of this code is based off of this article https://compendium.readthedocs.io/en/latest/tasks/drivetrains/mecanum.html
    public void calculateDrivePowers(double x, double y, double r) {

        double translationAngle = Math.atan2(y,x);
        double translationPower = Math.hypot(x,y);
        double turnPower = r;

        // calculate motor power
        double ADPower = translationPower * Math.sqrt(2) * 0.5 * (Math.sin(translationAngle) + Math.cos(translationAngle));
        double BCPower = translationPower * Math.sqrt(2) * 0.5 * (Math.sin(translationAngle) - Math.cos(translationAngle));

        // check if turning power will interfere with normal translation
        // check ADPower to see if trying to apply turnPower would put motor power over 1.0 or under -1.0
        double turningScale = Math.max(Math.abs(ADPower + turnPower), Math.abs(ADPower - turnPower));
        // check BCPower to see if trying to apply turnPower would put motor power over 1.0 or under -1.0
        turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + turnPower), Math.abs(BCPower - turnPower)));

        // adjust turn power scale correctly
        if (Math.abs(turningScale) < 1.0)
        {
            turningScale = 1.0;
        }

        // set the motors, and divide them by turningScale to make sure none of them go over the top, which would alter the translation angle
        leftFrontPower = ((ADPower - turningScale) / turningScale);
        leftRearPower = ((BCPower - turningScale) / turningScale);
        rightFrontPower = ((BCPower + turningScale) / turningScale);
        rightRearPower = ((ADPower + turningScale) / turningScale);

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



    // Hardcoded movement based on time
    public void move(double x, double y, double seconds) {
        move(x, y, seconds, false);
    }

    // Hardcoded movement based on time; if gyro is true, the robot will maintain it's current angle
    public void move(double x, double y, double seconds, boolean gyro) {
        if (gyro) {
            double t = elapsedSecs();
            while (elapsedSecs() - t < seconds) {
                calculateDrivePowers(x, y, getGyroPIDValue());
                sendDrivePowers();
            }
        } else {
            calculateDrivePowers(x, y, 0);
            sendDrivePowers();
            wait(seconds);
        }
        stopDrive();
    }

    // Resets the timer
    public void resetElapsedTime() {
        elapsedTime.reset();
    }

    // Sets current angle as zero
    public void resetGyroAngle() {
        gyro.resetAngle();
    }

    // Makes the robot rotate to a certain angle
    // maxFineTuning is the max number of seconds the robot can spend fine tuning to the correct angle
    public void rotateToPos(double angle, double maxFineTuning) {
        setTargetGyroAngle(angle);
        gyroPID.resetValues();
        double t = elapsedSecs();
        while(getAbsoluteGyroError() > 5 && elapsedTime.seconds() - t < 5) {
            adjustAngle();
        }
        t = elapsedSecs();
        gyroPID.resetValues();
        while ((getAbsoluteGyroError() > 1)
                && elapsedTime.seconds() - t < maxFineTuning) {
            adjustAngle();
        }
        stopDrive();
    }

    // Sends power to drive motors
    public void sendDrivePowers() {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
    }

    // Sends specific driver powers to drive motors
    public void sendDrivePowers(double x, double y, double rot) {
        calculateDrivePowers(x, y, rot);
        sendDrivePowers();
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
        sendDrivePowers(0, 0, 0);
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
