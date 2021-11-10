//WELCOME TO THE BEN BRANCH!

package org.firstinspires.ftc.teamcode.robot_components.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.data.HSVConstants;

// Robot class with more functionality than just the DriveBase
// Contains all of the motors/servos/sensors specific to this year's challenge
public class Robot extends DriveBase implements HSVConstants, FieldPositions {

    // Robot variables and objects
    //protected double spinnerPower = 0; maybe delete?
    protected double intakePower = 0;
    double dropperAngle;
    double intakeAngle;
    //public CRServo intakeCRServo;
    public DcMotor extenderMotor;
    public DcMotor intakeMotor;
    private final double EXTENDER_TICKS_PER_REV_OUTPUT_SHAFT = 384.5; // for 435 rpm yellowjacket
    private final double EXTENDER_PULLEY_INNER_CIRC = 36.0 * Math.PI; // very important for accurate distance!
    public DcMotor spinnerMotor;
    public Servo dropperServo;
    public Servo intakeBucketFlipServo;
    private final double DROPPER_MAX = 0.8;
    private final double DROPPER_MIN = 0.35;
    private final double INTAKE_DWN = 0;
    private final double INTAKE_UP = 0.9;

    // Constructs a robot with the mechanical functions specific to this year's competition
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap, telemetry); // Calls the DriveBase constructor, which handles drive motors
    }

}
