package org.firstinspires.ftc.teamcode.robot_components.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Robot class with more functionality than just the DriveBase
// Contains all of the motors/servos/sensors specific to this year's challenge
public class Robot extends DriveBase {

    // Robot variables and objects
    //protected double spinnerPower = 0; maybe delete?
    protected double intakePower = 0;
    protected double extenderPower = 0;
    protected double spinnerPower = 0;
    double dropperAngle;
    double intakeAngle;
    public double targetPosition = 0;

    //public CRServo intakeCRServo;
    public DcMotor extenderMotor;
    public static DcMotor intakeMotorFront;
    public static DcMotor intakeMotorBack;
    private final double EXTENDER_TICKS_PER_REV_OUTPUT_SHAFT = 384.5; // for 435 rpm yellowjacket
    private final double EXTENDER_PULLEY_INNER_CIRC = 36.0 * Math.PI; // very important for accurate distance!
    private final double ARM_SPEED = 50;
    public DcMotor spinnerMotor;
    public CRServo spinnerServoRed;
    public CRServo spinnerServoBlue;
    public Servo dropperServo;
    //public Servo spinnerServo;
    public Servo intakeBucketFlipServo;
    public static final double DROPPER_FORWARD = 0.3; //Maybe increase this to 0.6 or so
    public static final double DROPPER_NEUTRAL = 0.5;
    public static final double DROPPER_BACK = 0.76; //This value was previously 0.7, and was increased to 0.76
    public double DROPPER_CURRENT = 0;
    public static final double EXT_OUT = -3700; //Previous value was 2500, corrected to -3700
    public static final double EXT_LOW = -1600; //previous was -1500
    public static final double EXT_IN = -10; //Previous value was 0, corrected to -10

    // Constructs a robot with the mechanical functions specific to this year's competition
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap, telemetry); // Calls the DriveBase constructor, which handles drive motors

        intakeMotorFront = hardwareMap.get(DcMotor.class, "intakeMotorFront");
        //^Intake motor front also contains y-axis odometer in encoder slot
        intakeMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorBack = hardwareMap.get(DcMotor.class, "intakeMotorBack");
        intakeMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
        extenderMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperServo = hardwareMap.get(Servo.class, "dropperServo"); //need testing
        spinnerMotor = hardwareMap.get(DcMotor.class, "odo");
        //^Spinner motor (which doesn't exist anymore) also contains x-axis odometer in encoder slot
        spinnerServoRed = hardwareMap.get(CRServo.class, "spinnerServoRed");
        spinnerServoBlue = hardwareMap.get(CRServo.class, "spinnerServoBlue");
        dropperServo.setPosition(DROPPER_NEUTRAL);
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

    public double moveExtenderTo(double target) {
        extenderMotor.setTargetPosition((int)target);
        extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extenderMotor.setPower(1);
        return extenderMotor.getCurrentPosition();
    }

    public void moveExtenderWithJoyStick(double input) {

        double armPos = extenderMotor.getCurrentPosition();

        if(armPos >= EXT_OUT) {
            targetPosition = EXT_OUT;
        } else if(armPos <= EXT_IN) {
            targetPosition = EXT_IN;
        }
        else {
            targetPosition += input * ARM_SPEED;
        }

        moveExtenderTo(targetPosition);
    }

    public void setExtenderPower(double power){

        int position = extenderMotor.getCurrentPosition();


        while(position < EXT_IN) {
            position = extenderMotor.getCurrentPosition();
            extenderMotor.setPower(.2);
        }

        while(position > EXT_OUT) {
            position = extenderMotor.getCurrentPosition();
            extenderMotor.setPower(-.2);
        }

        extenderPower = power;
        extenderMotor.setPower(extenderPower);
    }


    /**
     * Toggle the position of dropperServo between DROPPER_MAX and DROPPER_MIN
     */

    /* KENNY PLS READ
    //just to clarify, the "bucket" is the part that is responsible for
    //holding the game element
    //the "intake" is responsible for catching game elements
    //I have updated the method names accordingly
    */
    //*~INTAKE FUNCTIONS*~//
    //manages up/down positions of intake

    public void setFrontIntakePower(double power){
        intakePower = power;
        intakeMotorFront.setPower(intakePower);
        //moveDropperCorrectly(power);
    }
    public void setBackIntakePower(double power){
        intakePower = power;
        intakeMotorBack.setPower(intakePower);
        //moveDropperCorrectly(-1 * power);
    }
    public void setSpinnerPower(double power){ //THIS IS OLD CODE WHEN WE ONLY HAD 1 SPINNER
        spinnerPower = power;
        spinnerMotor.setPower(spinnerPower);
    }

    public void spinRedDirection(double power){
        spinnerServoRed.setPower(-1 * power);
        spinnerServoBlue.setPower(power);
    }

    public void spinBlueDirection(double power){
        spinnerServoBlue.setPower(-1 * power);
        spinnerServoRed.setPower(power);
    }

    public void moveDropperCorrectly(double direction) {
        telemetry.addData("dropper position: ", dropperServo.getPosition());
        telemetry.update();
        if (direction < 0 && dropperServo.getPosition() != DROPPER_BACK) {
            this.backDropperPosition();
        } else if (direction > 0 && dropperServo.getPosition() != DROPPER_FORWARD) {
            this.forwardDropperPosition();
        }
    }

    public boolean intakeHasElement() {

        return false;
    }

    public void forwardDropperPosition() {
        dropperServo.setPosition(DROPPER_FORWARD);
    }

    public void backDropperPosition() {
        dropperServo.setPosition(DROPPER_BACK);
    }

    public void neutralDropperPosition() { dropperServo.setPosition(DROPPER_NEUTRAL); }

}
