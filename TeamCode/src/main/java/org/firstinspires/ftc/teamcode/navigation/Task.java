package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

public interface Task {

    public boolean update(RobotPosition currentPosition, Robot robot);


}
