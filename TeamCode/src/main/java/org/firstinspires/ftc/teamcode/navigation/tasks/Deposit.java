package org.firstinspires.ftc.teamcode.navigation.tasks;

import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;
import org.opencv.core.Rect;

/**
 * Deposit freight into an the chosen goal (this includes extension, drop, and retraction)
 */
public class Deposit implements Task {
    private boolean useBarcode;
    private boolean extended;
    private boolean dropped;
    private boolean returned;
    private double targetDist;
    private double initialTime;

    //private final double TICKSTODIST = 0.09363; //Multiply by this to convert from ticks to mm
    private static final double ERROR_MARGIN = 50;
    private static final double EXTENDED_WAIT_TIME = 1.5; //In seconds

    public Deposit() {
        this(Robot.EXT_OUT);
    }

    public Deposit(double targetDist) { //Full extension = -3700
        useBarcode = false;
        extended = false;
        dropped = false;
        returned = false;
        this.targetDist = targetDist;
    }

    public Deposit(boolean useBarcode) {
        this.useBarcode = useBarcode;
        extended = false;
        dropped = false;
        returned = false;
        this.targetDist = targetDist;
    }

    public void init() {}

    public boolean update(RobotPosition currentPosition, Robot robot) {
        int barX = robot.getBarcodePos().x;
        if (useBarcode) {
            targetDist = (barX < 110) ? Robot.EXT_LOW : (barX < 210)
                    ? Robot.EXT_MED : (barX < 320) ? Robot.EXT_OUT : 0;
        }
        if (targetDist == 0) {
            return true;
        }
        if (!extended) {
            double extendedDist = robot.moveExtenderTo((int)(targetDist));
            robot.telemetry.update();
            robot.neutralDropperPosition();
            if (Math.abs(extendedDist - targetDist) < ERROR_MARGIN) {
                extended = true;
            }
        } else if (!dropped) {
            robot.dropperServo.setPosition(1);
            initialTime = robot.elapsedSecs();
            dropped = true;
        } else if ((robot.elapsedSecs() - initialTime) > EXTENDED_WAIT_TIME) {
            double extendedDist = robot.moveExtenderTo((int)Robot.EXT_IN);
            robot.neutralDropperPosition();
            if (Math.abs(extendedDist - Robot.EXT_IN) < ERROR_MARGIN) {
                returned = true;
            }
        }
        return (extended && dropped && returned);
    }
}