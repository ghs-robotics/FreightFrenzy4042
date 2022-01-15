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
    public double direction;
    public double targetDist;
    public double startingDist;
    public boolean startingDistSet;
    public Telemetry telemetry;
    public boolean reachedTarget;

    public Intake(double direction, double targetDist, Telemetry telemetry) {
        this.direction = direction;
        this.targetDist = targetDist;
        this.telemetry = telemetry;
        reachedTarget = false;
        startingDistSet = false;
    }

    public void init() {}

    public boolean update(RobotPosition currentPosition, Robot robot) {
        double distance = currentPosition.position.y;
        if (!startingDistSet) {
            startingDist = distance;
            startingDistSet = true;
        }
        telemetry.addData("Reached target? ", reachedTarget);
        telemetry.addData("Distance: ", distance);
        if (!reachedTarget) {
            if (distance < targetDist) {
                robot.calculateDrivePowersAuto(0, direction * 1, 0);
                robot.sendDrivePowers();
                robot.setFrontIntakePower(direction);
                robot.setBackIntakePower(-1 * direction);
                robot.forwardDropperPosition();
            } else {
                reachedTarget = true;
            }
        } else {
            if (distance > startingDist) {
                robot.calculateDrivePowersAuto(0, direction * -1, 0);
                robot.sendDrivePowers();
                robot.setFrontIntakePower(-1);
                robot.setBackIntakePower(-1);
            } else {
                robot.calculateDrivePowersAuto(0, 0, 0);
                robot.sendDrivePowers();
                robot.setFrontIntakePower(0);
                robot.setBackIntakePower(0);
                robot.neutralDropperPosition();
                return true;
            }
        }
        return false;
    }
}