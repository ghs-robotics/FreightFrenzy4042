package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

import java.util.List;

public class AutoController {
    private List<Task> tasks;
    private int currentTaskIdx = 0;
    private Robot robot;
    private final double TICKSPERROT = 232.16;
    private final double ROTSPERTURN = 9.525;
    private final double WHEELCIRC = 150.8; //Circumference of the wheel given a 48mm diameter
    public Telemetry telemetry;

    public RobotPosition currentPosition;
    public RobotPosition startingPosition;

    public AutoController(Telemetry telemetry,  Robot robot) {
        this.telemetry = telemetry;
        this.robot = robot;
    }

    /**
     * Initialize with a series of points
     * @param tasks the series of points to drive to. probably in an ArrayList
     * @param startingPosition The starting locationd and orientation of the robot
     */
    public void initialize(List<Task> tasks, RobotPosition startingPosition) {
        this.tasks = tasks;
        this.startingPosition = startingPosition;
        this.currentPosition = startingPosition;
    }

    /**
     * continue running nav
     * @return true if done
     */
    public boolean update() { // call this to run nav
        currentPosition = updateCurrentPosition();
        if (currentTaskIdx >= tasks.size()) {
            return true; // done -> return true
        }
        // not done...
        Task currentTask = this.tasks.get(currentTaskIdx);
        boolean done = currentTask.update(currentPosition, robot);
        telemetry.addData("Done?", done);
        telemetry.update();
        if (done) switchToNextTask();
        return false;
    }

    public RobotPosition updateCurrentPosition() { //Wheels rotate 1.6 times slower (factored in already)
        //Gives the number of encoder ticks (232.16 ticks is 1 rotation)
        double degreesTurned = robot.gyro.getAngle(); //Get gyro angle to subtract from robot rotations
        double leftFrontTicks = robot.leftFrontDrive.getCurrentPosition();
        double leftFrontRotations = leftFrontTicks / TICKSPERROT;
        double rightFrontTicks = robot.rightFrontDrive.getCurrentPosition();
        double rightFrontRotations = rightFrontTicks / TICKSPERROT;
        //Must account for rotations that were used to re-orient the robot...
        //^One full 360 degree turn would require an estimated 9.525 wheel rotations
        //This assumes the robot has only underwent clockwise rotation of no more than 360 degrees total
        leftFrontRotations -= degreesTurned * (ROTSPERTURN / 360);
        rightFrontRotations += degreesTurned * (ROTSPERTURN / 360);
        //Calculate final x and y distances from the robot's starting position
        double xDist = (WHEELCIRC * Math.cos(Math.PI / 4)) * (leftFrontRotations - rightFrontRotations);
        double yDist = (WHEELCIRC * Math.sin(Math.PI / 4)) * (leftFrontRotations + rightFrontRotations);

        telemetry.addData("position", "xDist:" + xDist + "yDist:" + yDist);
        return new RobotPosition(startingPosition.position.add(new Point2D(xDist, yDist)),
                startingPosition.rotation);
    }

    private void switchToNextTask() {
        currentTaskIdx++;
        this.tasks.get(currentTaskIdx).init();
    }

    public void setTasks(List<Task> tasks) {
        this.tasks = tasks;
    }
}