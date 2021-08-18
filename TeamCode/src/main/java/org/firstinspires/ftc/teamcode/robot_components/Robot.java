package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.data.HSVConstants;

// Robot class with more functionality than just the DriveBase
// Contains all of the motors/servos/sensors specific to this year's challenge
public class Robot extends DriveBase implements HSVConstants, FieldPositions {

    public static final double DOWN_WHISKER_ANGLE = 0.4;
    public static final double UP_WHISKER_ANGLE = 0.02;

    // Robot variables and objects
    protected double intakePower = 0;
    public double armAngle = 0.15; // init position
    public double clawAngle = 0.80;// Closed position
    public double whiskerAngle = UP_WHISKER_ANGLE; // Whisker is up

    public DcMotor intakeMotor;
//    public CRServo intakeCRServo;
    public Servo armServo;
    public Servo clawServo;
    public Servo whiskerServo;

    public PowerLauncher powerLauncher; // Controls launching and indexing

    // Constructs a robot with the mechanical functions specific to this year's competition
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap, telemetry); // Calls the DriveBase constructor, which handles drive motors

        // These are the names to use in the phone config (in quotes below)
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
//        intakeCRServo = hardwareMap.get(CRServo.class, "intakeCRServo");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        whiskerServo = hardwareMap.get(Servo.class, "whiskerServo");

        powerLauncher = new PowerLauncher(hardwareMap); // Initializes powerLauncher
    }

    // Launches a ring by moving the indexerServo; this method is SLOW intentionally
    public void indexRings(int rings) {
        double time = 0.4;
        if (rings >= 2) {
            time = 0.25;
        }
        for (int i = 0; i < rings; i++) {
            powerLauncher.index(time);
            wait(time);
        }
    }

    // Sets servos to starting positions in init
    public void resetServos() {
        clawServo.setPosition(clawAngle);
        whiskerServo.setPosition(whiskerAngle);
        wait(0.6);
        armServo.setPosition(armAngle);
        powerLauncher.resetServos();
    }

    // Run intake with specified power; negative values make intake run backward
    public void runIntake(double power) {
        intakePower = power;
        intakeMotor.setPower(intakePower);
//        intakeCRServo.setPower(-intakePower * 0.5);
    }

    // Turn arm to specified position
    private void setArmAngle(double armAngle) {
        this.armAngle = armAngle;
        armServo.setPosition(armAngle);
    }

    // Toggles the wobble gripper/claw
    public void toggleClaw() {
        // Default angle is 0.85 (which means the gripper is closed)
        clawAngle = (clawAngle != 0.80 ? 0.80 : 0.35);
        clawServo.setPosition(clawAngle);
    }

    // Turns the arm
    public void turnArm() {
        // Default angle is 0.42 (which is the up position)
        setArmAngle(armAngle != 0.42 ? 0.42 : 0.88);
    }

    // Good position for grabbing a wobble goal
    public void turnArmDownFull() {
        setArmAngle(0.88);
    }

    // Good position for placing a wobble goal during auto
    public void turnArmDownSlight() {
        setArmAngle(0.80);
    }

    // Good position for holding the wobble goal right above ground
    public void turnArmDownDrag() {
        setArmAngle(0.75);
    }

    // For holding the wobble above the wall height
    public void turnArmUpFull() {
        setArmAngle(0.42);
    }

    public void turnWhiskerServo() {
        whiskerAngle = (whiskerAngle != UP_WHISKER_ANGLE ? UP_WHISKER_ANGLE : DOWN_WHISKER_ANGLE);
        whiskerServo.setPosition(whiskerAngle);
    }

    public void turnWhiskerUp() {
        whiskerAngle = UP_WHISKER_ANGLE;
        whiskerServo.setPosition(whiskerAngle);
    }

    public void turnWhiskerDown() {
        whiskerAngle = DOWN_WHISKER_ANGLE;
        whiskerServo.setPosition(whiskerAngle);
    }
}
