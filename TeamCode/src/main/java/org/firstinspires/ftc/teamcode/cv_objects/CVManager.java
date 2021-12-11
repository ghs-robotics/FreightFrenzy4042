package org.firstinspires.ftc.teamcode.cv_objects;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.navigation.AutoController;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CVManager {

    private OpenCvWebcam webcam;
    private CVPipeline pipeline;
    private WebcamName webcamName;
    private Robot robot;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private boolean cameraOn;

    public CVManager(HardwareMap hM, Telemetry t) {
        hardwareMap = hM;
        telemetry = t;
        cameraOn = false;
    }

    public void startCamera() {
        cameraOn = true;
    }

    public void stopCamera() {
        cameraOn = false;
    }

    public void init() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        pipeline= new CVPipeline();
        final OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                while(true) {
                    while (cameraOn) {
                        camera.setPipeline(pipeline);
                        camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                        // Usually this is where you'll want to start streaming from the camera (see section 4)
                    }
                }
            }
            public void onError(int errorCode) {/* This will be called if the camera could not be opened*/}
        });
    }

    public void cameraTelemetry() {
        telemetry.addData("Faces", pipeline.returnResultsFaces());
        telemetry.addData("Boxes", pipeline.returnResultsBoxes());
        telemetry.addData("Wiffles", pipeline.returnResultsWiffles());
        telemetry.addData("Ducks", pipeline.returnResultsDucks());
        telemetry.update();
    }

    public int detectBarcode() { //currently returns the x and not an x converted to barcode
        return pipeline.returnResultsFaces().toArray()[0].x;
    }
}