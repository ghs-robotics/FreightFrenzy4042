package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

import java.util.List;

public class AutoController {
    private List<Task> tasks;
    private int currentTaskIdx = 0;
    private Robot robot;

    public RobotPosition currentPosition;

    /**
     * Initialize with a series of points
     * @param tasks the series of points to drive to. probably in an ArrayList
     * @param startingPosition The starting locationd and orientation of the robot
     */
    public void initialize(List<Task> tasks, RobotPosition startingPosition) {
        this.tasks = tasks;
        this.currentPosition = startingPosition;
    }

    /**
     * continue running nav
     * @return true if done
     */
    public boolean update() { // call this to run nav
        if (currentTaskIdx >= tasks.size()) {
            return true; // done -> return true
        }
        // not done...
        Task currentTask = this.tasks.get(currentTaskIdx);
        boolean done = currentTask.update(currentPosition, robot);
        if (done) currentTaskIdx++;
        return false;
    }
}
