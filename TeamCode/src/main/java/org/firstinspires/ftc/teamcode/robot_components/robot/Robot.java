package org.firstinspires.ftc.teamcode.robot_components.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    protected double extenderPower = 0;
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
    private final double DROPPER_MAX = -0.8;
    private final double DROPPER_MIN = -0.35;
    private final double INTAKE_DWN = 0;
    private final double INTAKE_UP = 0.9;

    // Constructs a robot with the mechanical functions specific to this year's competition
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap, telemetry); // Calls the DriveBase constructor, which handles drive motors

        spinnerMotor = hardwareMap.get(DcMotor.class, "spinnerMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        extenderMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dropperServo = hardwareMap.get(Servo.class, "dropperServo");
        intakeBucketFlipServo = hardwareMap.get(Servo.class, "intakeServo");

        dropperServo.setPosition(DROPPER_MIN);
        intakeBucketFlipServo.setPosition(INTAKE_DWN);

        intakeAngle = INTAKE_DWN;
        dropperAngle = DROPPER_MIN;
    }

    /**
     * Toggle the extension between extended to a given length or retracted. Expects extenderMotor
     * to be set up properly with RunMode.RUN_TO_POSITION.
     * todo what unit of measurement is distance????? Gonna assume that it is in mm
     * @param distance The distance to extend to if retracted.
     */
    public void toggleExtension(double distance) {
        int currentTicks = extenderMotor.getCurrentPosition();
        boolean isExtended = currentTicks > 5 || currentTicks < -5;
        if (isExtended) {
            // retract
            extenderMotor.setTargetPosition(0);
        } else {
            // extend
            int targetTicks = (int) Math.round(
                    (distance / EXTENDER_PULLEY_INNER_CIRC /* num. revolutions*/)
                    * EXTENDER_TICKS_PER_REV_OUTPUT_SHAFT
            );
            extenderMotor.setTargetPosition(targetTicks);
        }
    }

    public void setExtenderPower(double power){
        extenderPower = power;
        extenderMotor.setPower(extenderPower);
    }


    /**
     * Toggle the position of dropperServo between DROPPER_MAX and DROPPER_MIN
     */
    public void dropGameElement() {
//        dropperAngle = (dropperAngle != 0.80 ? 0.80 : 0.35);
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
    public void toggleBucket(){
        boolean intakeIsDown = Math.abs(intakeAngle - INTAKE_DWN) < 0.001;
        intakeAngle = (intakeIsDown ? INTAKE_UP : INTAKE_DWN);
        intakeBucketFlipServo.setPosition(intakeAngle);
    }

    public void setIntakePower(double power){
        intakePower = power;
        intakeMotor.setPower(intakePower);
    }
}
