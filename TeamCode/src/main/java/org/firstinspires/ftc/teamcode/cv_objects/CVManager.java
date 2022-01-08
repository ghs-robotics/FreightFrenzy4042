package org.firstinspires.ftc.teamcode.cv_objects;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.navigation.AutoController;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class CVManager {

    private CVPipeline pipeline;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private OpenCvCamera webcam;
    private ElapsedTime elapsedTime;

    public CVManager(HardwareMap hM, Telemetry t) {
        hardwareMap = hM;
        telemetry = t;

        elapsedTime = new ElapsedTime();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        pipeline = new CVPipeline();
        webcam.setPipeline(pipeline);
    }

    public void initCamera() {
        while (elapsedTime.seconds() < 5) { }
        telemetry.addData("Camera is ", "initializing");
        telemetry.update();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addData("Camera is ", "opened");
                telemetry.update();
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("error ", errorCode);
                telemetry.update();
            }
        });
        elapsedTime.reset();
    }

    public void cameraTelemetry() {
        telemetry.addData("Faces", pipeline.returnResultsFaces().toList());
        /*telemetry.addData("Boxes", pipeline.returnResultsBoxes().toList().toString());
        telemetry.addData("Wiffles", pipeline.returnResultsWiffles().toList().toString());
        telemetry.addData("Ducks", pipeline.returnResultsDucks().toList().toString()); */
        telemetry.addData("InitFrame ", pipeline.returnInitialFrame().get(0, 0)[0]);
        telemetry.addData("FPS ", webcam.getFps());
        //telemetry.addData("file exists? ", pipeline.fileExists);
        telemetry.update();
    }

    public int detectBarcode() { //currently returns the x and not an x converted to barcode
        return pipeline.returnResultsFaces().toArray()[0].x;
    }
}