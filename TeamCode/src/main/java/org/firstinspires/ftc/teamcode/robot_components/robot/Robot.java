package org.firstinspires.ftc.teamcode.robot_components.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.data.HSVConstants;

// Robot class with more functionality than just the DriveBase
// Contains all of the motors/servos/sensors specific to this year's challenge
public class Robot extends DriveBase implements HSVConstants, FieldPositions {

    // Robot variables and objects
    //public CRServo intakeCRServo;
    public Servo thingDropper;
    double dropperAngle;
    // Constructs a robot with the mechanical functions specific to this year's competition
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap, telemetry); // Calls the DriveBase constructor, which handles drive motors


        thingDropper = hardwareMap.get(Servo.class, "thingDropper");

    }

    public void dropThings() {
        dropperAngle = (dropperAngle != 0.80 ? 0.80 : 0.35);
        thingDropper.setPosition(dropperAngle);
    }


}
