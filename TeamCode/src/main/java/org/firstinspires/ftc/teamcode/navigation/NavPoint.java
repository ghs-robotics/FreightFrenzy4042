package org.firstinspires.ftc.teamcode.navigation;

public class NavPoint {
    // Where to go. if the position or rotation is null, don't use it.
    public RobotPosition point;
    // How close the robot needs to be before tasks can be run. again, null means any distance
    public RobotPosition errorMargin;

    // TODO: tasks

    public NavPoint(RobotPosition point, RobotPosition errorMargin) {
        this.point = point;
        this.errorMargin = errorMargin;
    }
}
