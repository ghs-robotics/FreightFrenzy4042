package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

import java.util.List;

public class Nav {
    private List<NavPoint> points;
    private int currentPointIdx = 0;
    private Robot robot;

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

    }
}
