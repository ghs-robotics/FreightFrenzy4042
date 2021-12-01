package org.firstinspires.ftc.teamcode.robot_components.robot;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Optional;

public class SimpleDuckSpinner {
    CRServo spinnerServo;
    private ElapsedTime runtime;
    private Telemetry telemetry;
    private boolean isRunning = false;
    //Should be adjusted experimentally
    final double SERVO_SPEED = 0.5;
    //O means not running, 1 means forward, and -1 means running backwards
    private double servoDirection = 0;
    // TODO: SET DIRECTION !!!!

    public SimpleDuckSpinner(CRServo spinnerServo, double radius) {
        this(spinnerServo, radius, null);
    }

    public SimpleDuckSpinner(
            CRServo spinnerServo,
            double radius,
            Telemetry telemetry //,
    ){
        this.spinnerServo = spinnerServo;
        spinnerServo.setDirection(CRServo.Direction.FORWARD);
        this.runtime = new ElapsedTime();
        this.telemetry = telemetry;
    }

    public void startRunningForwards() {
        runtime.reset();
        servoDirection = 1;
        isRunning = true;
    }
    public void startRunningBackwards() {
        runtime.reset();
        servoDirection = -1;
        isRunning = true;
    }
    public void stopRunning() {
        runtime.reset();
        servoDirection = 0;
        isRunning = false;
    }
    /**
     * call this repeatedly to run the spinner
     * @return true if done spinning, else false
     */
    public void update() {
        double time = runtime.seconds();
        if(telemetry != null) {
            telemetry.addData("duck time: ", time);
            telemetry.addData("duck motor power: ", spinnerServo.getPower());
            telemetry.addData("Goal motor power: ", SERVO_SPEED * servoDirection);
        }
        spinnerServo.setPower(SERVO_SPEED * servoDirection);
    }
}