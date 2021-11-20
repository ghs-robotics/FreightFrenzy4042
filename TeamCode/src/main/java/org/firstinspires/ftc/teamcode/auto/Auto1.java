package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto_helpers.Nav;
import org.firstinspires.ftc.teamcode.cv_objects.CVPipeline;
import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.navigation.AutoController;
import org.firstinspires.ftc.teamcode.navigation.Point2D;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.navigation.tasks.Deposit;
import org.firstinspires.ftc.teamcode.navigation.tasks.DriveToPoint;
import org.firstinspires.ftc.teamcode.robot_components.cv.CVModule;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.robot_components.cv.CameraManager;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

//Identify barcode and deliver initial shipping element (~3 seconds) +20 points
//Collect and deliver 3 freight to alliance shipping hub (~18 seconds) +18 points
//Spin 1 duck (~6 seconds) +10 points
//Park in warehouse (~3 seconds) +10 points

@Autonomous
public class Auto1 extends LinearOpMode implements FieldPositions {

    // Declare OpMode members
    private CVModule robot;
    private AutoController autoController = new AutoController();
    //private OpenCvWebcam webcam;
    //private CVPipeline pipeline;
    //private WebcamName webcamName;

    public void initializeTasks() {
        List<Task> tasks = new ArrayList<>();
        //SCAN BARCODE AND PLACE INITIAL ELEMENT IF POSSIBLE
        tasks.add(drive(0, 500, 0.0));
        tasks.add(new Deposit());
        tasks.add(drive(0, 0, 0));
        //tasks.add(drive(0, 0, 90.0)); No need to turn 90 probably???
        tasks.add(drive(1000, 0, 0.0));

        autoController.setTasks(tasks);
    }

//TODO idk if the camera should be initialized here
    /*public void initializeCV() {
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
            public void onError(int errorCode) {/* This will be called if the camera could not be opened*///}
        /*});
        waitForStart();
    } */

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
        //initializeCV();
        initializeTasks();
        waitForStart();

        while(opModeIsActive()) {
            //telemetry.addData("Boxes", pipeline.returnResultsBoxes());
            //telemetry.addData("Wiffles", pipeline.returnResultsWiffles());
            //telemetry.addData("Ducks", pipeline.returnResultsDucks());
            //telemetry.update();
            autoController.update();
        }
    }
}
