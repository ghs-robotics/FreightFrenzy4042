package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv_objects.CVPipeline;
import org.firstinspires.ftc.teamcode.navigation.AutoController;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.cv.CVModule;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;


@Autonomous
public class Auto1 extends LinearOpMode {

    // Declare OpMode members
    private CVModule robot;
    private AutoController autoController = new AutoController(); //had to do context actions and get rid of a parameter in order for the code to build
    private OpenCvWebcam webcam;
    private CVPipeline pipeline;
    private WebcamName webcamName;

    public void initializeTasks() {
        List<Task> tasks = new ArrayList<>();
        //tasks.add(new DriveToPoint());
        autoController.setTasks(tasks);
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

    @Override
    public void runOpMode()
    {
        initializeCV();
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Boxes", pipeline.returnResultsBoxes());
            //telemetry.addData("Wiffles", pipeline.returnResultsWiffles());
            //telemetry.addData("Ducks", pipeline.returnResultsDucks());
            telemetry.update();
            sleep(100);
        }
    }
}

