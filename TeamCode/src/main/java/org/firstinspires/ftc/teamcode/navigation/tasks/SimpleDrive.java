package org.firstinspires.ftc.teamcode.navigation.tasks;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navigation.Point2D;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

public class SimpleDrive implements Task {
    private ElapsedTime timer;
    private Point2D direction;
    private double rotation;
    private double driveTime;
    private final double ROTATION_P_COEF = 1.0 / 60.0; // if > 60* off, full power

    public SimpleDrive(Point2D direction, double rot, double time) {
	this.direction = direction;
	this.rotation = rot;
	this.driveTime = time;
	this.timer = new ElapsedTime();
    }

    public SimpleDrive(double x, double y, double rot, double time) {
		this(new Point2D(x, y), rot, time);
    }

    public void init() {
	timer.reset();
    }

    public boolean update(RobotPosition currentPosition, Robot robot) {
	// done -> stop motors and return true
	if (timer.seconds() > driveTime) {
	    robot.calculateDrivePowers(0, 0, 0);
	    robot.sendDrivePowers();
	    return true;
	}
	// If not pointing the right direction, do that first.
	double rotDifference = robot.gyro.getAngle() % 360.0 - rotation;
	if (Math.abs(rotDifference) > 5.0) {
	    double rotPower = rotDifference * ROTATION_P_COEF;
	    robot.calculateDrivePowers(0, 0, rotPower);
	    robot.sendDrivePowers();
	    return false;
	}
	// time ok, rotation ok -> drive.
	robot.calculateDrivePowers(direction.x, direction.y, 0);
	robot.sendDrivePowers();
	return false;
    }
}
