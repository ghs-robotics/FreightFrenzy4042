package org.firstinspires.ftc.teamcode.robot_components.robot;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Optional;

public class SimpleDuckSpinner {
    DcMotor spinnerMotor;
    private ElapsedTime runtime;
    private Telemetry telemetry;
    private boolean isRunning = false;
    //Should be adjusted experimentally
    final double SERVO_SPEED = 0.5;
    //O means not running, 1 means forward, and -1 means running backwards
    private double servoDirection = 0;
    // TODO: SET DIRECTION !!!!

    public SimpleDuckSpinner(DcMotor spinnerMotor, double radius) {
        this(spinnerMotor, radius, null);
    }

    public SimpleDuckSpinner(
            DcMotor spinnerMotor,
            double radius,
            Telemetry telemetry //,
    ){
        this.spinnerMotor = spinnerMotor;
        spinnerMotor.setDirection(DcMotor.Direction.FORWARD);
        this.runtime = new ElapsedTime();
        this.telemetry = telemetry;
    }

    public void startRunningForwards() {
        runtime.reset();
        servoDirection = 1;
        spinnerMotor.setPower(1);
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

    public boolean isPowered () {
        return isRunning;
    }

    public double getTimeRunning() {
        return isRunning ? runtime.seconds() : -1.0;
    }
    /**
     * call this repeatedly to run the spinner
     * @return true if done spinning, else false
     */
    public void update() {
        double time = runtime.seconds();
        if(telemetry != null) {
            telemetry.addData("duck time: ", time);
            telemetry.addData("duck motor power: ", spinnerMotor.getPower());
            telemetry.addData("Goal motor power: ", SERVO_SPEED * servoDirection);
        }
        spinnerMotor.setPower(SERVO_SPEED * servoDirection);
    }
}