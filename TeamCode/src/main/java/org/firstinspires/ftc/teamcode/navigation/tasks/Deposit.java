package org.firstinspires.ftc.teamcode.navigation.tasks;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.navigation.Point2D;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

/**
 * Drive to a point on the field.
 */
public class Deposit implements Task {

    private boolean extended;
    private boolean dropped;
    private boolean lifted;
    private boolean returned;
    private boolean holdingElement;
    private double targetDist;
    private double initialTime;

    //private final double TICKSTODIST = 0.09363; //Multiply by this to convert from ticks to mm
    private static final double FULLY_RETRACTED = -10;
    private static final double FULLY_EXTENDED = -3700;
    private static final double ERROR_MARGIN = 50;
    private static final double EXTENDED_WAIT_TIME = 3; //In seconds

    public Deposit() {
        this(FULLY_EXTENDED);
    }

    public Deposit(double targetDist) { //Full extension = -3700
        extended = false;
        dropped = false;
        lifted = false;
        returned = false;
        holdingElement = true;
        this.targetDist = targetDist;
    }

    public void init() {}

    public boolean update(RobotPosition currentPosition, Robot robot) {
        if (!extended) {
            double extendedDist = robot.moveEntenderTo((int)(targetDist));
            robot.neutralDropperPosition();
            if (Math.abs(extendedDist - targetDist) < ERROR_MARGIN) {
                extended = true;
            }
        } else if (!dropped) {
            robot.dropperServo.setPosition(1);
            initialTime = robot.elapsedSecs();
            dropped = true;
        }
        if ((robot.elapsedSecs() - initialTime) > EXTENDED_WAIT_TIME) {
            double extendedDist = robot.moveEntenderTo((int)FULLY_RETRACTED);
            if (Math.abs(extendedDist - FULLY_RETRACTED) < ERROR_MARGIN) {
                returned = true;
            }
        }
        return (extended && dropped && returned);
    }
}