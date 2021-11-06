package org.firstinspires.ftc.teamcode.navigation.tasks;

import org.firstinspires.ftc.teamcode.navigation.Point2D;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

/**
 * Drive to a point on the field.
 */
public class DriveToPoint implements Task {

    public RobotPosition targetPosition;
    public RobotPosition errorMargin;

    public DriveToPoint(RobotPosition targetPosition) {
        // default error margin of 1.5cm, 10 degrees
        this(targetPosition, new RobotPosition(15.0, 15.0, 10.0));
    }

    public DriveToPoint(RobotPosition targetPosition, RobotPosition errorMargin) {
        this.targetPosition = targetPosition;
        this.errorMargin = errorMargin;
    }


    public void init() {
        // don't need to do anything
    }

    public boolean update(RobotPosition currentPosition, Robot robot) {

        Point2D error = targetPosition.position.subtract(currentPosition.position);
        double rotError = targetPosition.rotation - currentPosition.rotation;

        // todo: this is kinda jank and should use PID or something
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
