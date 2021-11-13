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
    final double MOTOR_SPEED = 0.8;
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
        this.telemetry = Optional.ofNullable(telemetry);

//        this.useEncoder = useEncoders;
    }

    public void startRunning() {
        runtime.reset();
        isRunning = true;
    }

    public void stopRunning() {
        motor.setPower(0);
        isRunning = false;
    }
    /**
     * call this repeatedly to run the spinner
     * @return true if done spinning, else false
     */
    public boolean update() {
        double time = runtime.seconds();
        telemetry.ifPresent(telemetry1 -> { // if telemetry is null nothing will happen
            telemetry1.addData("duck time", time);
            telemetry1.addData("duck motor power", motor.getPower());
            if (isRunning) {
                telemetry1.addData("Goal motor power", MOTOR_SPEED);
            } else {
                telemetry1.addData("Goal motor power", 0);
            }
        });
        if (isRunning) {
            motor.setPower(MOTOR_SPEED);
            return false;
        } else {
            // setting power to zero will make the motor brake and stop
            // TODO: use encoder to make sure the wheel doesn't slip to slow down as fast as possible
            this.stopRunning();
            return true;
        }
    }
}
