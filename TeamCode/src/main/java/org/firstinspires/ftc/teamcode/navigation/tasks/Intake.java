package org.firstinspires.ftc.teamcode.navigation.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.IntakeDetector;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

/**
 * Intake freight (requires detection of some kind)
 */
public class Intake implements Task {
    public IntakeDetector detector;
    public boolean intakeFinished;
    public double direction;
    public Telemetry telemetry;

    public Intake(double d, Telemetry t) {
        detector = new IntakeDetector(t);
        intakeFinished = false;
        direction = d;
        telemetry = t;
    }

    public void init() {}

    public boolean update(RobotPosition currentPosition, Robot robot) {
        double data = detector.update();
        telemetry.addData("data", data);
        /*telemetry.addData("posBack", data[1]);
        telemetry.addData("velFront", data[2]);
        telemetry.addData("velBack", data[3]); */
        telemetry.update();
        robot.calculateDrivePowersOffset(1,1,0,45);
        robot.sendDrivePowers();
        robot.setFrontIntakePower(direction);
        robot.setBackIntakePower(-1 * direction);
        robot.forwardDropperPosition();
        intakeFinished = detector.holdingObject();
        if (data > 0) {
            robot.calculateDrivePowersOffset(0, 0, 0, 45);
            robot.sendDrivePowers();
            robot.setFrontIntakePower(0);
            robot.setBackIntakePower(0);
        }
        return data > 0;
    }
}