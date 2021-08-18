package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.data.HSVConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.concurrent.CopyOnWriteArrayList;

// Manages the cameras on the robot (in this case the internal phone camera and one webcam)
// IMPORTANT NOTE: WHILE CAMERAS ARE STREAMING, THERE MAY BE LOTS OF LAG IN TELEOP!!
// That lag can be avoided by either not streaming or setting the waitTime higher in
// our pipelines (thus decreasing FPS)
public class CameraManager implements HSVConstants {

    private int cameraMonitorViewId;
    private boolean streaming = false; // true if cameras are streaming image input

    // Cameras
    public OpenCvCamera phoneCam;
    public OpenCvCamera webcam;

    // Pipelines for image processing
    public CVDetectionPipeline phoneCamPipeline;
    public CVDetectionPipeline webcamPipeline;

    HardwareMap hardwareMap;
    ElapsedTime elapsedTime;

    // Constructs a CameraManager with two cameras
    public CameraManager(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;
        elapsedTime = new ElapsedTime();

        // Initializes some CV variables/objects
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());


        // Uncomment ONE of the below statements ONLY
        // These determine what you will be able to see on the DS phone but do not affect CV functionality

//        setUpDualPort(hardwareMap);
//        setUpSinglePhonePort(hardwareMap);
        setUpSingleWebcamPort(hardwareMap);
//        setUpNoStreamPort(hardwareMap);

        // Opens cameras
        phoneCam.openCameraDevice();
        webcam.openCameraDevice();

        // Creates and assigns each camera a pipeline
        phoneCamPipeline = new CVDetectionPipeline();
        webcamPipeline = new CVDetectionPipeline();

        // Initializes the ArrayLists in each pipeline
        // We use CopyOnWriteArrayLists to avoid any ConcurrentModificationExceptions
        phoneCamPipeline.activeObjects = new CopyOnWriteArrayList<>();
        webcamPipeline.activeObjects = new CopyOnWriteArrayList<>();

        // Choose the pipelines in which image processing will occur for each respective camera
        phoneCam.setPipeline(phoneCamPipeline);
        webcam.setPipeline(webcamPipeline);
    }

    // Initializes the camera
    public void initCamera() {
        startStreaming();
    }

    // Returns true if cameras are streaming
    public boolean isStreaming() {
        return streaming;
    }

    // Webcam takes some time to boot up
    public boolean isWebcamReady() {
        return elapsedTime.seconds() > 1.7;
    }

    // Displays both webcam and phoneCam streams on the DS phone
    private void setUpDualPort(HardwareMap hardwareMap) {
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, // The container we're splitting
                        2, // The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);


        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[0]);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class,"Webcam 1"), viewportContainerIds[1]);
    }

    // Displays nothing on the DS phone
    private void setUpNoStreamPort(HardwareMap hardwareMap) {
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class,"Webcam 1"));
    }

    // Displays only the phoneCam stream on the DS phone
    private void setUpSinglePhonePort(HardwareMap hardwareMap) {
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class,"Webcam 1"));
    }

    // Displays only the webcam stream on the DS phone
    private void setUpSingleWebcamPort(HardwareMap hardwareMap) {
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
    }

    // Starts streaming frames on both cameras
    public void startStreaming() {
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        streaming = true;
        elapsedTime.reset();
    }

    // Start streaming in the middle of a match
    // Ideally you won't need to use this because starting a stream takes a lot of time
    // A way around this is to never stop streaming in the first place but instead
    // increase the waitTime in CVDetectionPipeline whenever CV is not actively being used
    public void startMidStream() {
        if (!streaming) {
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            webcam = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, "Webcam 1"));
            webcam.setPipeline(webcamPipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
            });
            webcam.openCameraDevice();
            streaming = true;
            elapsedTime.reset();
        }
    }

    // Stops streaming frames on the phone camera
    public void stopStreaming() {
        streaming = false;
        phoneCam.stopStreaming();
        webcam.stopStreaming();
    }
}