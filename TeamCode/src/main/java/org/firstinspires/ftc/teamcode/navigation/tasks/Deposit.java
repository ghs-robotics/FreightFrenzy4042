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

    private final double TICKSTODIST = 0.09363; //Multiply by this to convert from ticks to mm

    public Deposit(double targetDist) { //Full extension dist = 300
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
            double extendedDist = (double)robot.toggleExtension(targetDist) * TICKSTODIST;
            if (Math.abs(extendedDist - targetDist) < 50) {
                extended = true;
            }
        } else if (!dropped) {
            robot.dropGameElement();
            dropped = true;
        } else if (!lifted) {
            if (robot.dropperIsMin()) {
                robot.dropGameElement();
                holdingElement = false;
            }
            if (robot.dropperIsMax() && !holdingElement) {
                lifted = true;
            }
        } else {
            double extendedDist = robot.toggleExtension(-targetDist) * TICKSTODIST;
            if (Math.abs(extendedDist + targetDist) < 50) {
                returned = true;
            }
        }
        return (extended && dropped && lifted && returned && !holdingElement);
    }
}