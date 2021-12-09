package org.firstinspires.ftc.teamcode.navigation.tasks;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.navigation.Point2D;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

/**
 * Drive to a point on the field.
 * @warning THIS IS PROBABLY BROKEN
 */
public class DriveToPoint implements Task {

    public RobotPosition targetPosition;
    public RobotPosition errorMargin;
    public Telemetry telemetry;

    public DriveToPoint(RobotPosition targetPosition) {
        // default error margin of 1.5cm, 10 degrees
        this(targetPosition, new RobotPosition(15.0, 15.0, 10.0));
    }

    public DriveToPoint(RobotPosition targetPosition, RobotPosition errorMargin) {
        this.targetPosition = targetPosition;
        this.errorMargin = errorMargin;
        //driveTime = targetPosition.position.length() / 762.5;

    }


    public void init() {
        // don't need to do anything
    }

    public boolean update(RobotPosition currentPosition, Robot robot) {

        telemetry = robot.telemetry;

        telemetry.addData("cx: ", currentPosition.position.x);
        telemetry.addData("tx: ", targetPosition.position.x);
        telemetry.addData("cy: ", currentPosition.position.y);
        telemetry.addData("ty: ", targetPosition.position.y);

        telemetry.update();

        Point2D error = targetPosition.position.subtract(currentPosition.position);
        Point2D errorPID = error.scale(0.00170).exponent(4);
        //0.00145 = 0.93, 0.00150 = 0.96, 0.00160 = 0.98

        double rotError = targetPosition.rotation - currentPosition.rotation;
        double rotErrorPID = rotError * 0.01;

        // todo: this is kinda jank and should use PID or something

        boolean arrived = arrived(currentPosition);

        // TODO THIS DOESN'T REALLY WORK MOST LIKELY

        if (!arrived) {
            robot.calculateDrivePowersOffset(Math.max(-1, Math.min(1, errorPID.x)),
                    Math.max(-1, Math.min(1, errorPID.y)), Math.max(-1, Math.min(1, rotErrorPID)), 120);
            robot.sendDrivePowers();
        } else {
            robot.calculateDrivePowersOffset(0, 0, 0, 120);
            robot.sendDrivePowers();
        }

        return arrived;
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
