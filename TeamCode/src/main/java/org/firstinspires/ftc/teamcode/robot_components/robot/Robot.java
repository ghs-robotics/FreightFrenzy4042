package org.firstinspires.ftc.teamcode.robot_components.robot;

import com.qualcomm.robotcore.hardware.CRServo;
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
    double dropperTargetAngle;
    protected double extenderPower = 0;
    double intakeAngle;
    //public CRServo intakeCRServo;
    public DcMotor extenderMotor;
    public DcMotor intakeMotor;
    //public DcMotor odometerMotor;
    private final double EXTENDER_TICKS_PER_REV_OUTPUT_SHAFT = 384.5; // for 435 rpm yellowjacket
    private final double EXTENDER_PULLEY_INNER_CIRC = 36.0 * Math.PI; // very important for accurate distance!
    public DcMotor spinnerMotor;
    public CRServo spinnerServo;
    public Servo dropperServo;
    //public Servo spinnerServo;
    public Servo intakeBucketFlipServo;
    private final double DROPPER_MAX = 0.45; //Maybe increase this to 0.6 or so
    private final double DROPPER_MIN = 0.3; //Need testing
    private final double INTAKE_DWN = 0;
    private final double INTAKE_UP = 0.9;
    private final double EXT_OUT = 2500;
    private final double EXT_IN = 0;
    private final double ODOM_TICKS_PER_ROT = 232.16; //Placeholder. Figure this out!
    private final double ODOM_CIRCUMFERENCE = 150.8; //Circumference of the odometer given a 48mm diameter
    public final double ODOM_TICKS_TO_DIST = ODOM_CIRCUMFERENCE / ODOM_TICKS_PER_ROT;
    //Trial results:

    // Constructs a robot with the mechanical functions specific to this year's competition
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap, telemetry); // Calls the DriveBase constructor, which handles drive motors

        //spinnerMotor = hardwareMap.get(DcMotor.class, "spinnerMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        extenderMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        //odometerMotor = hardwareMap.get(DcMotor.class, "odometerMotor");
       // extenderMotor.setTargetPosition(0);
       // extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dropperServo = hardwareMap.get(Servo.class, "dropperServo"); //need testing
        spinnerServo = hardwareMap.get(CRServo.class, "spinnerServo");
        //spinnerServo = hardwareMap.get(Servo.class, "spinnerServo");


        dropperServo.setPosition(DROPPER_MIN);
        //intakeBucketFlipServo.setPosition(INTAKE_DWN);

        intakeAngle = INTAKE_DWN;
        dropperTargetAngle = DROPPER_MIN;
    }

    /**
     * Toggle the extension between extended to a given length or retracted. Expects extenderMotor
     * to be set up properly with RunMode.RUN_TO_POSITION.
     * todo what unit of measurement is distance????? Gonna assume that it is in mm
     * @param distance The distance to extend to if retracted.
     */
    //Should return current distance extended
    public double toggleExtension(double distance) { //300 is the minimum required for purple bucket to be off robot

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
        return currentTicks;
    }

    /*public double getOdomPosition() {
        double distance = odometerMotor.getCurrentPosition() * ODOM_TICKS_TO_DIST;
        if (distance != 0) {
            return distance;
        } else {
            return 3.4404;
        }
    } */

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
    public void dropGameElement() {
        //dropperAngle = (dropperAngle != 0.80 ? 0.80 : 0.35);
        //Important note: never compare doubles or floats with == or !=, because floating point error
        dropperTargetAngle = (dropperIsMax() ? DROPPER_MIN : DROPPER_MAX); // toggle
        dropperServo.setPosition(dropperTargetAngle);
    }

    public boolean dropperIsMax() { //return a number between 0 and 1
        return Math.abs(dropperServo.getPosition() - DROPPER_MAX) < 0.1;
    }

    public boolean dropperIsMin() {
        return Math.abs(dropperServo.getPosition() - DROPPER_MIN) < 0.1;
    }

    public void jankToggleDropper() {
        dropperServo.setPosition(dropperTargetAngle);
        dropperTargetAngle = dropperTargetAngle == DROPPER_MIN ? DROPPER_MAX : DROPPER_MIN;
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

    public void duckSpinnerJank(double power) {
        spinnerMotor.setPower(power);
    }

    public void setIntakePower(double power){
        intakePower = power;
        intakeMotor.setPower(intakePower);
    }
}
