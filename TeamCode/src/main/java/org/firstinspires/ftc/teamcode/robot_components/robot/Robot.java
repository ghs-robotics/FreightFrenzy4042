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
    protected double spinnerPower = 0;
    double dropperAngle;
    //public CRServo intakeCRServo;
    public DcMotor extenderMotor;
    private final double EXTENDER_TICKS_PER_REV_OUTPUT_SHAFT = 384.5; // for 435 rpm yellowjacket
    private final double EXTENDER_PULLEY_INNER_CIRC = 36.0 * Math.PI; // very important for accurate distance!
    public DcMotor spinnerMotor;
    public Servo dropperServo;
    private final double DROPPER_MAX = 0.8;
    private final double DROPPER_MIN = 0.35;

    // Constructs a robot with the mechanical functions specific to this year's competition
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap, telemetry); // Calls the DriveBase constructor, which handles drive motors

        spinnerMotor = hardwareMap.get(DcMotor.class, "spinnerMotor");
        dropperServo = hardwareMap.get(Servo.class, "dropperServo");
        extenderMotor = hardwareMap.get(DcMotor.class, "extenderMotor");
        extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Toggle the extension between extended to a given length or retracted. Expects extenderMotor
     * to be set up properly with RunMode.RUN_TO_POSITION.
     * @param distance The distance to extend to if retracted.
     */
    public void toggleExtension(double distance) {
        int currentTicks = extenderMotor.getCurrentPosition();
        if (currentTicks > 5 || currentTicks < -5) { // could also use math.abs but this is simple
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

    /**
     * Toggle the position of dropperServo between DROPPER_MAX and DROPPER_MIN
     */
    public void dropThings() {
//        dropperAngle = (dropperAngle != 0.80 ? 0.80 : 0.35);
        // Important note: never compare doubles or floats with == or !=, because floating point error
        dropperAngle = (Math.abs(dropperAngle - DROPPER_MAX) < 0.001 ? DROPPER_MAX : DROPPER_MIN);
        dropperServo.setPosition(dropperAngle);
    }

    // This probably shouldn't be used since there is a dedicated duckSpinner class? idk
    public void duckSpin(double power){
        spinnerPower = power;
        spinnerMotor.setPower(spinnerPower);

    }




}
