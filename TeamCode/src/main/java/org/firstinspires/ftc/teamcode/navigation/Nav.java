package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

import java.util.List;

public class Nav {
    private List<NavPoint> points;
    private int currentPointIdx = 0;
    private Robot robot;

    /**
     * Initialize with a series of points
     * @param points the series of points to drive to. probably in an ArrayList
     */
    public void initialize(List<NavPoint> points) {
        this.points = points;
    }

    /**
     * continue running nav
     * @return true if done
     */
    public boolean update() { // call this to run nav
        if (currentPointIdx >= points.size()) {
            return true; // done -> return true
        }
        // not done...
        NavPoint currentPoint = this.points.get(currentPointIdx);
        boolean arrived = driveToPoint(currentPoint);
        if (arrived) currentPointIdx++;
        return false;
    }

    private boolean driveToPoint(NavPoint point) {
        // TODO: pathing algorithm.

        // check if done
        if (
            inRange(point.point.position.x, currentPosition.x, point.errorMargin.position.x) &&
            inRange(point.point.position.y, currentPosition.y, point.errorMargin.position.y) &&
            inRange(point.point.rotation, currentRotation, point.errorMargin.rotation)
        ) {
            // done
            return true;
        }
        return false;
    }

    /**
     * check if a is within maxDist of b
     * @param a value a
     * @param b value b
     * @param maxDist max distance between a and b
     * @return if within dist
     */
    private boolean inRange(float a, float b, float maxDist) {
        return Math.abs(a - b) < maxDist;
    }
}
