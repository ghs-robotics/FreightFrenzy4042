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
    public DcMotor spinnerMotor;
    public Servo dropperServo;

    // Constructs a robot with the mechanical functions specific to this year's competition
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap, telemetry); // Calls the DriveBase constructor, which handles drive motors

        spinnerMotor = hardwareMap.get(DcMotor.class, "spinnerMotor");
        dropperServo = hardwareMap.get(Servo.class, "thingDropper");

    }

    public void dropThings() {
        dropperAngle = (dropperAngle != 0.80 ? 0.80 : 0.35);
        dropperServo.setPosition(dropperAngle);
    }

    public void duckSpin(double power){
        spinnerPower = power;
        spinnerMotor.setPower(spinnerPower);

    }




}
