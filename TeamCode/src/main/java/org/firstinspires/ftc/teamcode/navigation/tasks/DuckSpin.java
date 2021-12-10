package org.firstinspires.ftc.teamcode.navigation.tasks;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.navigation.Point2D;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.DuckSpinner;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;
import org.firstinspires.ftc.teamcode.robot_components.robot.SimpleDuckSpinner;

/**
 * Drive to a point on the field.
 */
public class DuckSpin implements Task {

    private int direction;
    private boolean spinSucceeded;
    private SimpleDuckSpinner spinner;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public DuckSpin(int direction, Robot robot) { //1 = forwards, -1 = backwards (direction currently does nothing
        spinSucceeded = false;
        this.hardwareMap = robot.hardwareMap;
        this.telemetry = robot.telemetry;
        CRServo duckServo = hardwareMap.get(CRServo.class, "spinnerServo");
        spinner = new SimpleDuckSpinner(duckServo, 1);
        telemetry.addData("servo: ", duckServo);
        telemetry.update();
        this.direction = direction;
        duckServo.setPower(1);
    }

    public void init() {}

    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean update(RobotPosition currentPosition, Robot robot) {
        double runtime = spinner.getRuntime();
        //telemetry.addData("Runtime: ", runtime);
        if (!spinSucceeded) {
            //telemetry.addData("Spin ", "not successful yet");
            if (!spinner.checkIfRunning()) {
                //telemetry.addData("Spinner ", "is not running");
                spinner.startRunningForwards();
            } else if (runtime > 5) {
                //telemetry.addData("Spinner ", "has stopped running");
                spinner.stopRunning();
                spinSucceeded = true;
            } else {
                //telemetry.addData("Spinner ", "is at power " + spinner.getPower());
            }
        }
        spinner.update();
        //telemetry.addData("Spin finished? ", spinSucceeded);
        //telemetry.update();
        return spinSucceeded;
    }
}