package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

public interface Task {

    /**
     * Called once when switching tasks. This can be used to set things up.
     */
    public void init();

    /**
     * Continue running this task.
     * @param currentPosition the current position and rotation of the robot
     * @param robot the robot object, so this task can drive/etc
     * @return true if this task is done, otherwise false
     */
    public boolean update(RobotPosition currentPosition, Robot robot);
}
