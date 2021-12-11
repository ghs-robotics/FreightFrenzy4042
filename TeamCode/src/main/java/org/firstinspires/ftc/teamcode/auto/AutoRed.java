package org.firstinspires.ftc.teamcode.auto;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv_objects.CVPipeline;
import org.firstinspires.ftc.teamcode.navigation.AutoController;
import org.firstinspires.ftc.teamcode.navigation.Point2D;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.navigation.tasks.Deposit;
import org.firstinspires.ftc.teamcode.navigation.tasks.DriveToPoint;
import org.firstinspires.ftc.teamcode.navigation.tasks.DuckSpin;
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
public class AutoRed extends LinearOpMode {

    // Declare OpMode members
    private Robot robot;
    private AutoController autoController = new AutoController(); //had to do context actions and get rid of a parameter in order for the code to build
    private OpenCvWebcam webcam;
    private CVPipeline pipeline;
    private WebcamName webcamName;
    private Object Spin;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void initializeTasks() {

        List<Task> tasks = new ArrayList<>();
        //manager.init();
        //int barcodeLevel = manager.detectBarcode(); //Currently setting to a position, not a level
        tasks.add(new Deposit());
        tasks.add(drive(-1200, -1200, 0.0));

        /*tasks.add(drive(1000, 1000, 0.0));
        tasks.add(new DuckSpin(robot, -1)); //Forwards = 1, backwards = -1
        tasks.add(drive(-1000, -1000, 0.0)); */
        //tasks.add(drive(0, 0, 0.0));
        //tasks.add(new Deposit(barcodeLevel * 100));
        //tasks.add(drive(0, -1000, 0.0));
        //tasks.add(new DuckSpin(1)); //Forwards = 1, backwards = -1
        //tasks.add(drive(1000, 0, 0.0));
        tasks.add(new Stop());

        //telemetry.addData("barcode", barcodeLevel);

        telemetry.addData("task 1", tasks.get(0));
        telemetry.update();
        autoController.setTasks(tasks);
        autoController.initialize(tasks, new RobotPosition(new Point2D(0, 0), 0.0));
    }

    //TODO idk if the camera should be initialized here
    public void initializeCV() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        pipeline= new CVPipeline();
        final OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                waitForStart();
                while(opModeIsActive()){
                    camera.setPipeline(pipeline);
                    camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                    // Usually this is where you'll want to start streaming from the camera (see section 4)
                }
            }
            public void onError(int errorCode) {/* This will be called if the camera could not be opened*/}
        });
    }

    DriveToPoint drive(double x, double y, double rot) {
        return new DriveToPoint(new RobotPosition(new Point2D(x, y), rot));
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry); // new CVModule(hardwareMap, telemetry);
        //initializeCV();
        waitForStart();
        autoController = new AutoController(hardwareMap, telemetry,robot);
        robot.elapsedTime.reset();
        initializeTasks();


        while(opModeIsActive()){
            //telemetry.addData("Boxes", pipeline.returnResultsBoxes());
            //telemetry.addData("Wiffles", pipeline.returnResultsWiffles());
            //telemetry.addData("Ducks", pipeline.returnResultsDucks());
            telemetry.update();
            autoController.update();
        }
    }
}