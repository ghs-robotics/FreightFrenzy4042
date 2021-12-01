
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv_objects.CVManager;
import org.firstinspires.ftc.teamcode.cv_objects.CVPipeline;
import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.navigation.AutoController;
import org.firstinspires.ftc.teamcode.navigation.Point2D;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.navigation.tasks.Deposit;
import org.firstinspires.ftc.teamcode.navigation.tasks.DriveToPoint;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.robot_components.cv.CVModule;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//Identify barcode and deliver initial shipping element (~3 seconds) +20 points
//Collect and deliver 3 freight to alliance shipping hub (~18 seconds) +18 points
//Spin 1 duck (~6 seconds) +10 points
//Park in warehouse (~3 seconds) +10 points

@Autonomous
public class Auto1 extends LinearOpMode implements FieldPositions {

    // Declare OpMode members
    private Robot robot;
    private AutoController autoController;
    private CVManager manager;

    public void initializeTasks() {

        List<Task> tasks = new ArrayList<>();
        //SCAN BARCODE AND PLACE INITIAL ELEMENT IF POSSIBLE
        //int barcodeLevel = manager.detectBarcode();
        //tasks.add(drive(0, 500, 0.0));
        //tasks.add(new Deposit(barcodeLevel * 100));
        //tasks.add(drive(-1000, 0, 0.0));
        //tasks.add(new DuckSpin(1)); //Forwards = 1, backwards = -1
        //tasks.add(drive(1000, 0, 0.0));

        telemetry.addData("task 1", tasks.get(0));
        telemetry.update();
        autoController.setTasks(tasks);
        autoController.initialize(tasks, new RobotPosition(new Point2D(0, 0), 0.0));
    }

    DriveToPoint drive(double x, double y, double rot, Telemetry telemetry) {
        return new DriveToPoint(new RobotPosition(new Point2D(x, y), rot));
    }

    public void onError(int errorCode) {/* This will be called if the camera could not be opened*/}
        /*});
        waitForStart();
    } }*/

    int detectBarcode() {
        int shippingX = 0;
        //Magically assign shippingX here
        return shippingX / 107;
    }

    DriveToPoint drive(double x, double y, double rot) {
        return new DriveToPoint(new RobotPosition(new Point2D(x, y), rot));
    }

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry);
        waitForStart();
        autoController = new AutoController(telemetry, robot);
        robot.elapsedTime.reset();
        manager = new CVManager(hardwareMap, telemetry);
        manager.startCamera();
        initializeTasks();

        while(opModeIsActive()) {
            manager.cameraTelemetry();
            autoController.update();
        }
    }
}
