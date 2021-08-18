package org.firstinspires.ftc.teamcode.robot_components.navigation;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OdometryModule {

    public DcMotor deadWheelX;
    public DcMotor deadWheelY;

    private double deadWheelXRaw;
    private double deadWheelYRaw;

    private double xChange;
    private double yChange;

    //use the gyroscope for getting angle

    //TODO parse dead wheel inputs
    private void rawInput2Distance() {

    }
}
