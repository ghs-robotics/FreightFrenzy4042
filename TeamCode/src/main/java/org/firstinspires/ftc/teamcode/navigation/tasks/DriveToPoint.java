package org.firstinspires.ftc.teamcode.navigation.tasks;

import org.firstinspires.ftc.teamcode.navigation.Point2D;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

public class DriveToPoint implements Task {

    public RobotPosition targetPosition;
    public RobotPosition errorMargin;

    public void init() {
        // don't need to do anything
    }

    public boolean update(RobotPosition currentPosition, Robot robot) {

        Point2D error = targetPosition.position.subtract(currentPosition.position);
        double rotError = targetPosition.rotation - currentPosition.rotation;

        // todo: this is kinda jank and should use PID
        robot.calculateDrivePowers(error.x, error.y, rotError);

        return arrived(currentPosition);
    }

    private boolean arrived(RobotPosition currentPosition) {
        return inRange(targetPosition.position.x, currentPosition.position.x, errorMargin.position.x)
                && inRange(targetPosition.position.y, currentPosition.position.y, errorMargin.position.y)
                && inRange(targetPosition.rotation, currentPosition.rotation, errorMargin.rotation);
    }

    /**
     * check if a is within maxDist of b
     * @param a value a
     * @param b value b
     * @param maxDist max distance between a and b
     * @return if within dist
     */
    private boolean inRange(double a, double b, double maxDist) {
        return Math.abs(a - b) < maxDist;
    }
}
