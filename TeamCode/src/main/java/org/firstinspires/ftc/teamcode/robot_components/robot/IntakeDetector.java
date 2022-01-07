package org.firstinspires.ftc.teamcode.robot_components.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeDetector {
    public DcMotor intakeMotorFront;
    public DcMotor intakeMotorBack;
    public double velocityFront;
    public double velocityBack;
    public double curPositionFront;
    public double prePositionFront;
    public double curPositionBack;
    public double prePositionBack;
    private boolean motorSpinning;
    private boolean pickedUpObject;
    public ElapsedTime elapsedTime;
    public Telemetry telemetry;
    public double preTime;

    public IntakeDetector(Telemetry t) {
        intakeMotorFront = Robot.intakeMotorFront;
        intakeMotorBack = Robot.intakeMotorBack;
        velocityFront = 0;
        velocityBack = 0;
        curPositionFront = 0;
        prePositionFront = 0;
        curPositionBack = 0;
        prePositionBack = 0;
        motorSpinning = false;
        pickedUpObject = false;
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        telemetry = t;
        preTime = elapsedTime.seconds();
    }

    public double update() {
        curPositionFront = intakeMotorFront.getCurrentPosition();
        curPositionBack = intakeMotorBack.getCurrentPosition();
        velocityFront = (curPositionFront - prePositionFront) / (elapsedTime.seconds() - preTime);
        velocityBack = (curPositionBack - prePositionBack) / (elapsedTime.seconds() - preTime);
        preTime = elapsedTime.seconds();
        /*if (velocityFront > 50 || velocityBack > 50) {
            motorSpinning = true;
        } */
        if (velocityFront > 0) { //motorSpinning && (velocityFront < 10 || velocityBack < 10)
            pickedUpObject = true;
        }

        prePositionFront = curPositionFront;
        prePositionBack = curPositionBack;

        return velocityFront;
        //return new double[]{curPositionFront, curPositionBack, velocityFront, velocityBack};
    }

    public boolean holdingObject() {
        return pickedUpObject;
    }
}
