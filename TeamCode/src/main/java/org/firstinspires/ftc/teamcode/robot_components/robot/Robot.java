package org.firstinspires.ftc.teamcode.robot_components.robot;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Robot class with more functionality than just the DriveBase
// Contains all of the motors/servos/sensors specific to this year's challenge
public class Robot extends DriveBase{

    // Robot variables and objects
    //protected double spinnerPower = 0; maybe delete?
    public static Telemetry telemetry;
    protected double intakePower = 0;
    protected double extenderPower = 0;
    double dropperAngle;
    double intakeAngle;
    //public CRServo intakeCRServo;
    public DcMotor extenderMotor;
    public DcMotor intakeMotor;
    private final double EXTENDER_TICKS_PER_REV_OUTPUT_SHAFT = 384.5; // for 435 rpm yellowjacket
    private final double EXTENDER_PULLEY_INNER_CIRC = 36.0 * Math.PI; // very important for accurate distance!
    public DcMotor spinnerMotor;
    public CRServo spinnerServo;
    public Servo dropperServo;
    public DigitalChannel limitSwitch;
    //public Servo spinnerServo;
    public Servo intakeBucketFlipServo;
    private final double DROPPER_MAX = 0.45; //Maybe increase this to 0.6 or so
    private final double DROPPER_MIN = 0.3; //Need testing
    private final double INTAKE_DWN = 0;
    private final double INTAKE_UP = 0.9;
    private final double EXT_OUT = 2500;
    private final double EXT_IN = 0;
    private final    int EXT_ERROR = 10;
    private final double EXT_SPEED = .5;

    // Constructs a robot with the mechanical functions specific to this year's competition
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap, telemetry); // Calls the DriveBase constructor, which handles drive motors

        Robot.telemetry = telemetry;

        //spinnerMotor = hardwareMap.get(DcMotor.class, "spinnerMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotorFront");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extenderMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        extenderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // extenderMotor.setTargetPosition(0);
       // extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dropperServo = hardwareMap.get(Servo.class, "dropperServo"); //need testing
        spinnerServo = hardwareMap.get(CRServo.class, "spinnerServo");
        //spinnerServo = hardwareMap.get(Servo.class, "spinnerServo");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.OUTPUT);

        dropperServo.setPosition(DROPPER_MIN);
        //intakeBucketFlipServo.setPosition(INTAKE_DWN);

        intakeAngle = INTAKE_DWN;
        dropperAngle = DROPPER_MIN;
    }

    /**
     * because RUN TO POSITION MODE does not seem to work, im gonna do a pro gamer move and
     * program it myself
     * - simon
     *
     * This method working by moving the arm to the intended target position
     *
     * target position should be within the extender's tick range
     *
     * @param targetPosition
     */
    public void toggleArm(int targetPosition) {

        if(targetPosition < EXT_IN || targetPosition > EXT_OUT) {
            telemetry.addData("ERROR: ", "target position is our of range!");
            telemetry.update();
            return;
        }

        int armPosition = extenderMotor.getCurrentPosition();

        if(Math.abs(targetPosition - armPosition) < EXT_ERROR) return;

        if(targetPosition < armPosition) {
            extenderMotor.setPower(EXT_SPEED);
        } else if(targetPosition < armPosition) {
            extenderMotor.setPower(-EXT_SPEED);
        }
    }

    public void retractArm() {
        while(!limitSwitch.getState()) {
            extenderMotor.setPower(-EXT_SPEED);
        }
    }

    /**
     * Toggle the position of dropperServo between DROPPER_MAX and DROPPER_MIN
     */
    public void dropGameElement() {
        //dropperAngle = (dropperAngle != 0.80 ? 0.80 : 0.35);
        // Important note: never compare doubles or floats with == or !=, because floating point error
        boolean dropperIsMax = Math.abs(dropperAngle - DROPPER_MAX) < 0.001;
        dropperAngle = (dropperIsMax ? DROPPER_MIN : DROPPER_MAX); // toggle
        dropperServo.setPosition(dropperAngle);
    }

    /* KENNY PLS READ
    //just to clarify, the "bucket" is the part that is responsible for
    //holding the game element
    //the "intake" is responsible for catching game elements
    //I have updated the method names accordingly
    */
    //*~INTAKE FUNCTIONS*~//
    //manages up/down positions of intake

    public void setIntakePower(double power){
        intakePower = power;
        intakeMotor.setPower(intakePower);
    }
}
