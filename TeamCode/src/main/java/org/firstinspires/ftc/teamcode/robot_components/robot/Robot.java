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

    // Constructs a robot with the mechanical functions specific to this year's competition
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap, telemetry); // Calls the DriveBase constructor, which handles drive motors

        spinnerMotor = hardwareMap.get(DcMotor.class, "spinnerMotor");
        dropperServo = hardwareMap.get(Servo.class, "thingDropper");
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
        if (currentTicks > 5 || currentTicks < -5) {
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

    public void dropThings() {
        dropperAngle = (dropperAngle != 0.80 ? 0.80 : 0.35);
        dropperServo.setPosition(dropperAngle);
    }

    // This probably shouldn't be used since there is a dedicated duckSpinner class? idk
    public void duckSpin(double power){
        spinnerPower = power;
        spinnerMotor.setPower(spinnerPower);

    }




}
