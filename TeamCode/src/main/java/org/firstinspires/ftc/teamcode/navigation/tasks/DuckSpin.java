package org.firstinspires.ftc.teamcode.navigation.tasks;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.navigation.Point2D;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

/**
 * Drive to a point on the field.
 */
public class DuckSpin implements Task {

    private int direction;
    private boolean spinSucceeded;

    public DuckSpin(int direction) { //1 = forwards, -1 = backwards
        spinSucceeded = false;
        this.direction = direction;
    }

    public void init() {}

    public boolean update(RobotPosition currentPosition, Robot robot) {

        return spinSucceeded;
    }
}

