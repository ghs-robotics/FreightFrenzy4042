package org.firstinspires.ftc.teamcode.auto;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv_objects.CVManager;
import org.firstinspires.ftc.teamcode.cv_objects.CVPipeline;
import org.firstinspires.ftc.teamcode.navigation.AutoController;
import org.firstinspires.ftc.teamcode.navigation.Point2D;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.navigation.tasks.Deposit;
import org.firstinspires.ftc.teamcode.navigation.tasks.DriveToPoint;
import org.firstinspires.ftc.teamcode.navigation.tasks.DuckSpin;
import org.firstinspires.ftc.teamcode.navigation.tasks.ScanCode;
import org.firstinspires.ftc.teamcode.navigation.tasks.Stop;
import org.firstinspires.ftc.teamcode.robot_components.cv.CVModule;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;


@Autonomous
public class AutoBlue extends LinearOpMode {

    // Declare OpMode members
    private Robot robot;
    private AutoController autoController = new AutoController(); //had to do context actions and get rid of a parameter in order for the code to build
    //private CVManager manager;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void initializeTasks() {

        List<Task> tasks = new ArrayList<>();
        tasks.add(new ScanCode(hardwareMap, telemetry));
        tasks.add(new DriveToPoint(true));
        tasks.add(new Deposit(true)); //Use barcode is set to true for this time
        //tasks.add(new Deposit());
        //tasks.add(drive(1364, 1364, 0.0));

        tasks.add(new Stop());

        //telemetry.addData("barcode", barcodeLevel);

        //telemetry.addData("task 1", tasks.get(0));
        //telemetry.update();
        autoController.setTasks(tasks);
        autoController.initialize(tasks, new RobotPosition(new Point2D(0, 0), 0.0));
    }

    DriveToPoint drive(double x, double y, double rot) {
        return new DriveToPoint(new RobotPosition(new Point2D(x, y), rot));
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode()
    {
        /*manager = new CVManager(hardwareMap, telemetry);
        manager.initCamera(); */
        robot = new Robot(hardwareMap, telemetry); // new CVModule(hardwareMap, telemetry);
        waitForStart();
        autoController = new AutoController(hardwareMap, telemetry,robot);
        robot.elapsedTime.reset();
        initializeTasks();

        while(opModeIsActive()){
            //manager.cameraTelemetry();
            /*telemetry.addData("map: ", ScanCode.getMap().toString());
            telemetry.update(); */
            autoController.update();
        }
    }
}