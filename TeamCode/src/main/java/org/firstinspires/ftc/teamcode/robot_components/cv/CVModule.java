// Deprecated
//Do Not Use!!!!


package org.firstinspires.ftc.teamcode.robot_components.cv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot_components.movement_enhancing.PIDController;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;

// The lowest class in the inheritance hierarchy; any OpModes should extend CVRobot
public class CVModule extends Robot{

    public CameraManager cameras; // Manages the webcam and phone camera

    // Objects to be detected through CV
    public CVObject target;

    // Used in several methods for regulating motor powers in automated functions
    protected double x = 0; // strafe power to be sent
    protected double y = 0; // forward-backward power to be sent

    protected double phaseTimeStamp = 0; // stores data for automated functions in TeleOp

    // PID controllers
    public PIDController towerXPID; // For the x-position of the robot relative to tower goal x
    public PIDController towerWPID; // For the y-position of the robot relative to tower goal width
    public PIDController xPID;
    public PIDController wPID;

    // Constructs a CVRobot object that uses computer vision (CV)
    public CVModule(HardwareMap hardwareMap, Telemetry telemetry) {

        // Calls the constructor in Robot, which handles all motors/servos/etc.
        super(hardwareMap, telemetry);

        cameras = new CameraManager(hardwareMap); // Sets up the phone camera and webcam

        // Initializing PID objects for each CV target object

        // When working together with wPID, having Ki and Kd be zero works best
        towerXPID = new PIDController(0.0400, 0.0015, 0.0000, 0, 0.26);

        // Having Ki and Kd be zero normally works fine though
        towerWPID = new PIDController(0.0450, 0.0010, 0.0000, 0, 0.15);

        xPID = new PIDController(0.0200, 0.0000, 0.0000, 1); // Could be better
        wPID = new PIDController(0.0250, 0.0000, 0.0000, 1); // Could be better

        CVDetectionPipeline web = cameras.webcamPipeline; // temporary
        CVDetectionPipeline phone = cameras.phoneCamPipeline; // temporary
    }


    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ------------------------------------   HELPER METHODS   -------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------

    // Call before using CV for field localization
    public void activateFieldLocalization() {
        startUp();
        CVDetectionPipeline.sleepTimeMS = 0;
        cameras.startMidStream(); // start streaming image input
        cameras.webcam.resumeViewport(); // show webcam images on phone in real time
    }

    public void startUp() {
        stopDrive();
        gyroPID.resetValues();
        CVDetectionPipeline.sleepTimeMS = 0; // increase FPS to maximum
        targetGyroAngle = getReasonableGyroAngle(0);
    }

    // Displays a bunch of useful values on the DS phone
    @Override
    public void addTelemetryData() {

        addCVTelemetryData();

        telemetry.addData("gyro angle", "" + gyro.getAngle());

        telemetry.addLine("sleepTimeMS: " + CVDetectionPipeline.sleepTimeMS);

        telemetry.update();
    }

    // Add certain optional data to telemetry
    private void addCVTelemetryData() {
    }

    // Number of seconds since the phaseTimeStamp variable was reset
    public double getPhaseTimePassed() {
        return elapsedSecs() - phaseTimeStamp;
    }

    // Call after using CV for field localization in order to reduce lag in teleop
    public void deactivateFieldLocalization() {
        stopDrive();
        CVDetectionPipeline.sleepTimeMS = 500;
        cameras.webcam.pauseViewport(); // stop displaying webcam stream on phone
    }
    
    // To use at the start of each OpMode that uses CV
    public void initWithCV() {
        stopDrive();
        cameras.initCamera();
        resetGyroAngle();
    }

    // Use at the start of each OpMode that does not use CV
    public void initWithoutCV() {
        initWithCV();
        cameras.stopStreaming();
        cameras.stopStreaming();
    }

    public void resetPhaseTimeStamp() {
        phaseTimeStamp = elapsedSecs();
    }

    // Helper method for setting up



}

    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------   AUTOMATED FUNCTIONS   -----------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------