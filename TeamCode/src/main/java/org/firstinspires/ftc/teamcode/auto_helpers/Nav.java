package org.firstinspires.ftc.teamcode.auto_helpers;

import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

public class Nav {
    Robot robot;
    double[] position;
    double[] velocity;
    double[] goalPos;


    public void init() {
        init(new double[]{0, 0});
    }

    public void init(double[] startingPos) {
        position = startingPos;
    }

    public void moveTo(double[] goalPos) {
        this.goalPos = goalPos;
    }

    public void updateVel() {
        velocity = robot.calculateDrivePowers(position[0], position[1], goalPos[0], goalPos[1]);
        robot.sendDrivePowers();
    }

    public void updatePos() { //Should be called once every 100ms
        position[0] += velocity[0] / 10;
        position[1] += velocity[1] / 10;
    }
}
