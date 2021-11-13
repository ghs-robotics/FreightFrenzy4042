package org.firstinspires.ftc.teamcode.robot_components.robot;

import android.os.Build;

import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Optional;

@RequiresApi(api = Build.VERSION_CODES.N) // needed to use Optional (not actually, but android studio complains)
public class SimpleDuckSpinner {
    DcMotor motor = null;
    private ElapsedTime runtime = null;
    private Optional<Telemetry> telemetry; // used to log power if not null
    // Optional<e> prevents NullReferenceExceptions

    private boolean isRunning = false;
    private int spinnerMotorDirection = 0;
    final double MOTOR_SPEED = 1;
    // TODO: SET DIRECTION !!!!

    public SimpleDuckSpinner(DcMotor motor) {

        this(motor, null);
    }

    public SimpleDuckSpinner(
            DcMotor motor,
            Telemetry telemetry //,
//            boolean useEncoders
    ){
        this.motor = motor;
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.runtime = new ElapsedTime();
        this.telemetry = Optional.ofNullable(telemetry);

//        this.useEncoder = useEncoders;
    }

    public void runForwards() {
        spinnerMotorDirection = 1;
    }
    public void runBackwards() {
        spinnerMotorDirection = -1;
    }

    public void stopRunning() {
        spinnerMotorDirection = 0;
    }
    /**
     * call this repeatedly to run the spinner
     * @return true if done spinning, else false
     */
    public void update() {
        double time = runtime.seconds();
        telemetry.ifPresent(telemetry1 -> { // if telemetry is null nothing will happen
            telemetry1.addData("duck time", time);
            telemetry1.addData("duck motor power", motor.getPower());
            /*
            if (isRunning) {
                telemetry1.addData("Goal motor power", MOTOR_SPEED);
            } else {
                telemetry1.addData("Goal motor power", 0);
            }
            */
        });
        motor.setPower(spinnerMotorDirection  * MOTOR_SPEED);
    }
}
