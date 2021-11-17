package org.firstinspires.ftc.teamcode.robot_components.robot;

import android.os.Build;

import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Optional;

public class SimpleDuckSpinner {
    DcMotor motor;
    private ElapsedTime runtime;
    private Telemetry telemetry;
    private boolean isRunning = false;
    //Should be adjusted experimentally
    final double MOTOR_SPEED = 0.5;
    //O means not running, 1 means forward, and -1 means running backwards
    private double motorDirection = 0;
    // TODO: SET DIRECTION !!!!

    public SimpleDuckSpinner(DcMotor motor, double radius) {
        this(motor, radius, null);
    }

    public SimpleDuckSpinner(
            DcMotor motor,
            double radius,
            Telemetry telemetry //,
//            boolean useEncoders
    ){
        this.motor = motor;
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.runtime = new ElapsedTime();
        this.telemetry = telemetry;

//        this.useEncoder = useEncoders;
    }

    public void startRunningForwards() {
        runtime.reset();
        motorDirection = 1;
        isRunning = true;
    }
    public void startRunningBackwards() {
        runtime.reset();
        motorDirection = -1;
        isRunning = true;
    }
    public void stopRunning() {
        runtime.reset();
        motorDirection = 0;
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
            telemetry.addData("duck motor power: ", motor.getPower());
            telemetry.addData("Goal motor power: ", MOTOR_SPEED * motorDirection);
        }
        motor.setPower(MOTOR_SPEED * motorDirection);
    }
}
