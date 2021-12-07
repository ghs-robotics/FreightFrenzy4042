package org.firstinspires.ftc.teamcode.navigation.tasks;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.navigation.Point2D;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.DuckSpinner;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

/**
 * Drive to a point on the field.
 */
public class DuckSpin implements Task {

    private int direction;
    private boolean spinSucceeded;
    private DuckSpinner spinner;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public DuckSpin(int direction) { //1 = forwards, -1 = backwards (direction currently does nothing
        spinSucceeded = false;
        DcMotor duckMotor = hardwareMap.get(DcMotor.class, "duckMotor");
        spinner = new DuckSpinner(duckMotor, 0.5, .1, telemetry);
        this.direction = direction;
    }

    public void init() {}

    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean update(RobotPosition currentPosition, Robot robot) {
        spinner.update();
        if (!spinSucceeded) {
            if (!spinner.isPowered()) {
                spinner.startRunning();
            } else if (spinner.getRuntime().seconds() > 5){
                spinner.stopRunning();
                spinSucceeded = true;
            }
        }
        return spinSucceeded;
    }
}

