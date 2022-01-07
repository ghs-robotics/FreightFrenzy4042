package org.firstinspires.ftc.teamcode.navigation.tasks;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

//Stop the robot entirely
public class Stop implements Task {

    public Telemetry telemetry;

    public Stop() {

    }

    public void init() {
        // don't need to do anything
    }

    public boolean update(RobotPosition currentPosition, Robot robot) {
        telemetry = robot.telemetry;
        telemetry.addData("cx: ", currentPosition.position.x);
        telemetry.addData("cy: ", currentPosition.position.y);
        telemetry.update();
        return false;
    }
}