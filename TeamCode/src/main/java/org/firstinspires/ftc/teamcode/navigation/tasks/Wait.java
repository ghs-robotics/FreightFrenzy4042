package org.firstinspires.ftc.teamcode.navigation.tasks;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

public class Wait implements Task {
    private ElapsedTime waitingTime = new ElapsedTime();
    Telemetry telemetry;
    double seconds;
    public Wait (double inputTime) {
        seconds = inputTime;
        waitingTime.reset();
        telemetry.addData("Status:", "Waiting");
        telemetry.update();
        while(waitingTime.seconds() < seconds) {
            telemetry.addData("Time Waiting:", waitingTime.seconds());
            telemetry.update();
        }
        telemetry.addData("Status:", "Finished");
        telemetry.update();
    }

    public void init() {
    }

    public boolean update (RobotPosition position, Robot robot) {
            //Uses Thread.sleep if  there is more than a seconds left to wait
        return (waitingTime.seconds() >= seconds);
    }
}