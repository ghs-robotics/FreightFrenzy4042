package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv_objects.CVObject;
import org.firstinspires.ftc.teamcode.cv_objects.TowerGoal;
import org.firstinspires.ftc.teamcode.cv_objects.WobbleGoal;
import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.data.HSVConstants;

// The lowest class in the inheritance hierarchy; any OpModes should extend CVRobot
public class CVRobot extends Robot implements HSVConstants, FieldPositions {

    public CameraManager cameras; // Manages the webcam and phone camera

    // Objects to be detected through CV
    public CVObject target;
    public FieldFloor floor;
    public FieldWall wall;
    public Ring ring;
    public StarterStack stack;
    public TowerGoal tower;
    public WobbleGoal wobble;

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
    public CVRobot(HardwareMap hardwareMap, Telemetry telemetry) {

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

        floor = new FieldFloor(phone, new PIDController(0.0600, 0.0035, 0.0020, 1)); // yPID
        wall = new FieldWall(phone, new PIDController(0.0300, 0.0020, 0.0000, 0, 0.115)); // hPID
        ring = new Ring(phone, xPID, wPID);
        stack = new StarterStack(web);
        tower = new TowerGoal(web, towerXPID, towerWPID);
        wobble = new WobbleGoal(phone, xPID, wPID);
        target = wobble;
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

    // Displays a bunch of useful values on the DS phone
    @Override
    public void addTelemetryData() {

        addCVTelemetryData();

        telemetry.addData("CURRENT LAUNCH ANGLE", "" + Math.round(1000 * powerLauncher.launchAngle));
        telemetry.addData("PERFECT LAUNCH ANGLE", "" + Math.round(1000 * powerLauncher.PERFECT_LAUNCH_ANGLE));
        telemetry.addData("gyro angle", "" + gyro.getAngle());

        telemetry.addLine("sleepTimeMS: " + CVDetectionPipeline.sleepTimeMS);

        double dist = tower.cvtH2VerticalDist();
        telemetry.addData("Dist", "" + dist);
        telemetry.addData("Launch offset", "" + tower.cvtFt2LaunchOffset(dist));

        telemetry.addLine();

        telemetry.addLine("" + target.toString());

        telemetry.addLine();

        telemetry.addData("TOWER", "" + tower.toString());
        telemetry.addData("WALL", "" + wall.toString());
        telemetry.update();
    }

    // Add certain optional data to telemetry
    private void addCVTelemetryData() {
        telemetry.addData("phonecam crosshair: ", cameras.phoneCamPipeline.crosshairHSV);
        telemetry.addData("webcam crosshair: ", cameras.webcamPipeline.crosshairHSV);
        telemetry.addLine();

        telemetry.addLine("STREAMING: " + cameras.isStreaming());
        telemetry.addLine("CONFIG: " + identifyRingConfig());

        telemetry.addLine("LeftRightError: " + tower.getLeftRightError(0));
        telemetry.addLine("Left side: " + tower.x);
        telemetry.addLine("Right side: " +  (320 - tower.x - tower.w));

        telemetry.addData("(x, y)", "( " + x + ", " + y + " )");
        telemetry.addData("Kp", target.depthPID.k_P);
        telemetry.addData("Ki", target.depthPID.k_I);
        telemetry.addData("Kd", target.depthPID.k_D);
    }

    // Number of seconds since the phaseTimeStamp variable was reset
    public double getPhaseTimePassed() {
        return elapsedSecs() - phaseTimeStamp;
    }

    // Call after using CV for field localization in order to reduce lag in teleop
    public void deactivateFieldLocalization() {
        stopDrive();
//        tower.deactivate();
//        wall.deactivate();
        tower.setToNotIdentified();
        wall.setToNotIdentified();
        CVDetectionPipeline.sleepTimeMS = 500;
        cameras.webcam.pauseViewport(); // stop displaying webcam stream on phone
    }

    // Classifies the starter stack
    public int identifyRingConfig() {
        return stack.findConfig();
    }

    // To use at the start of each OpMode that uses CV
    public void initWithCV() {
        stopDrive();
        cameras.initCamera();
        resetServos();
        resetGyroAngle();
        tower.activate();
        wall.activate();
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
    public void startUp() {
        stopDrive();
        tower.activate();
        wall.activate();
        tower.resetPIDs();
        wall.resetPIDs();
        gyroPID.resetValues();
        CVDetectionPipeline.sleepTimeMS = 0; // increase FPS to maximum
        targetGyroAngle = getReasonableGyroAngle(0);
    }




    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------   AUTOMATED FUNCTIONS   -----------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------


    // Makes the robot line up with the tower goal (if called repeatedly)
    // Make sure the following things are accounted for before calling this:
    // targetGyroAngle, target values, reset PIDs, activate object, set launcher side as front
    public void adjustPosition() {
        if (getAbsoluteGyroError() > 4) {
            adjustAngle();
        }
        // When robot is too close to front of field
        else if (!tower.isIdentified()) {
            chaseObject(wall);
        }
        else {
            chaseObject(tower);
        }
    }

    // Shoot from anywhere on field in auto
    public void autoShoot() {
        int phase = 20;
        while (phase > 0) {
            phase = autoShootInPhases(phase);
        }
    }

    // Automated move to position function that uses phases and must be called repeatedly
    // This allows us to terminate the function early (because we can just set phase to be 0)
    public int autoShootInPhases(int phase) {

        int offSet = 0;

        // Distance between tower goal and left side of screen: tower.x
        // Distance between tower goal and right side of screen: 320 - tower.x - tower.w
        double error = tower.getLeftRightError(offSet);

        if (phase >= 11) {
            activateFieldLocalization();
            tower.setTargetXW(LEFT_POWERSHOT_POS);
            targetGyroAngle = getReasonableGyroAngle(0);
            phase = 10;
        }

        switch (phase) {
            case 10:
                if (cameras.isStreaming() && cameras.isWebcamReady()) {
                    phase--;
                }
                powerLauncher.toggleOn();
                break;
            case 9:
                if (getAbsoluteGyroError() < 35 && tower.isIdentified()) {
                    resetPhaseTimeStamp();
                    phase--;
                } else {
                    adjustPosition();
                }
                break;
            case 8:
                if (Math.abs(error) <= 40 && getPhaseTimePassed() > 0.1) {
                    resetPhaseTimeStamp();
                    phase--;
                } else {
                    calculateDrivePowers(0, 0, tower.getRotPIDVal(offSet, 5));
                    sendDrivePowers();
                }
                break;
            case 7:
                if (Math.abs(error) <= 10 && getPhaseTimePassed() > 0.1) {
                    resetPhaseTimeStamp();
                    phase--;
                } else {
                    calculateDrivePowers(0, 0, tower.getRotPIDVal(offSet, 4));
                    if (tower.isIdentified() && Math.abs(error) < 20) {
                        setAssistedLaunchAngle();
                    }
                    sendDrivePowers();
                }
                break;
            case 6:
                if (Math.abs(error) <= (tower.h > 32 ? 10 : 3) && getPhaseTimePassed() > 0.1) {
                    stopDrive();
                    phase--;
                } else {
                    calculateDrivePowers(0, 0, tower.getRotPIDVal(offSet, 3));
                    setAssistedLaunchAngle();
                    sendDrivePowers();
                }
                break;
        }

        if (1 < phase && phase < 6) {
            stopDrive();
            phase = powerLauncher.handleIndexQueue(phase - 1) + 1;
            resetPhaseTimeStamp();
        }

        else if (phase == 1 && getPhaseTimePassed() > 0.5) {
            powerLauncher.toggleOff(); // Turn launcher off after indexing
            deactivateFieldLocalization();
            phase--;
        }

        return phase;
    }

    // Makes the robot chase the target object (if called repeatedly)
    // Make sure the following things are accounted for before calling this:
    // gyro angle, target values, reset PIDs, activate object, set launcher side as front
    public void chaseObject(CVObject target) {
        x = target.getBreadthPIDValue();
        y = target.getDepthPIDValue();
        calculateDrivePowers(x, y, getGyroPIDValue());
        updateDrive();
    }

    public int moveInPhases(int phase) {
        return moveInPhases(phase, 0.2, 1.2, 4.0);
    }

    // Automated move to position function that uses phases and must be called repeatedly
    // This allows us to terminate the function early (because we can just set phase to be 0)
    public int moveInPhases(int phase, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        if (phase > 4) {
            phase = 4;
        }
        switch (phase) {
            case 4:
                activateFieldLocalization();
                phase--;
                break;
            case 3:
                if (cameras.isStreaming() && cameras.isWebcamReady()) {
                    phaseTimeStamp = elapsedSecs();
                    phase--;
                }
                break;
            case 2:
                if ((!tower.isIdentified() || !tower.targetInRange(4) || getAbsoluteGyroError() > 4)
                        && elapsedSecs() - phaseTimeStamp < maxBroadTuning) {
                    adjustPosition();
                }
                else {
                    phaseTimeStamp = elapsedSecs();
                    phase--;
                }
                break;
            case 1:
                if (elapsedSecs() - phaseTimeStamp < minFineTuning
                        || (elapsedSecs() - phaseTimeStamp < maxFineTuning && !tower.targetInRange(1))) {
                    adjustPosition();
                }
                else {
                    deactivateFieldLocalization();
                    phase--;
                }
                break;
        }
        return phase;
    }

    public void moveToPos(int[] pos) {
        moveToPos(pos, 0.0, 2.0, 5.0, 2);
    }

    public void moveToPos(int[] pos, int tolerance) {
        moveToPos(pos, 0.0, 2.0, 5.0, tolerance);
    }

    public void moveToPos(int[] pos, double maxFineTuning) {
        moveToPos(pos, 0.0, maxFineTuning, 5.0, 2);
    }

    public void moveToPos(int[] pos, double maxFineTuning, int tolerance) {
        moveToPos(pos, 0.0, maxFineTuning, 5.0, tolerance);
    }

    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning) {
        moveToPos(pos, minFineTuning, maxFineTuning, 5.0, 2);
    }

    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning, int tolerance) {
        moveToPos(pos, minFineTuning, maxFineTuning, 5.0, tolerance);
    }

    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        moveToPos(pos, minFineTuning, maxFineTuning, maxBroadTuning, 2);
    }

    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning, double maxBroadTuning, int tolerance) {
        moveToPos(pos, minFineTuning, maxFineTuning, maxBroadTuning, tolerance, false);
    }

    // Makes the robot move to a certain position relative to the tower goal
    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning, double maxBroadTuning, int tolerance, boolean shooting) {
        tower.setTargetXW(pos);
        activateFieldLocalization();
        rotateToPos(0.0, 0.0);
        double t = elapsedSecs();

        while((!tower.isIdentified() || !tower.targetInRange(4) || getAbsoluteGyroError() > 4)
                && elapsedSecs() - t < maxBroadTuning) {
            adjustPosition();
            if (shooting && tower.h <= 39) {
                setAssistedLaunchAngle();
            }
        }
        t = elapsedSecs();
        while (elapsedSecs() - t < minFineTuning
                || (elapsedSecs() - t < maxFineTuning && !tower.targetInRange(tolerance))) {
            adjustPosition();
            if (shooting) {
                setAssistedLaunchAngle();
            }
        }
        stopDrive();
    }

    // Move to a certain distance from the back wall
    // Only call this if the robot is already at least 2 feet from the back wall!
    public void moveUsingFloor(int targetY, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        floor.activate();
        floor.setTargetY(targetY);
        rotateToPos(0.0, 0.0);
        double t = elapsedSecs();
        while(  (floor.getAbsErrorY() > 3 || getAbsoluteGyroError() > 4)
                && elapsedSecs() - t < maxBroadTuning) {
            chaseObject(floor);
        }
        t = elapsedSecs();

        // Start fresh by resetting this
        floor.resetPIDs();

        while (elapsedSecs() - t < minFineTuning || (elapsedSecs() - t < maxFineTuning && driveMotorsRunning())) {
            chaseObject(floor);
        }
        stopDrive();
    }

    // Move to a certain distance from the back wall
    // Only call this if the robot is already at least 2 feet from the back wall!
    public void moveUsingWall(int wallH, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        wall.activate();
        wall.setTargetH(wallH);
        rotateToPos(0.0, 0.0);
        double t = elapsedSecs();
        while(  (wall.getAbsErrorH() > 3 || getAbsoluteGyroError() > 4)
                && elapsedSecs() - t < maxBroadTuning) {
            chaseObject(wall);
        }
        t = elapsedSecs();

        // Start fresh by resetting this
        wall.resetPIDs();

        while (elapsedSecs() - t < minFineTuning || (elapsedSecs() - t < maxFineTuning && driveMotorsRunning())) {
            chaseObject(wall);
        }
        stopDrive();
    }

    // Makes robot move forward and pick up wobble goal
    public void pickUpWobbleGoal(String finalArmPos) {
        stopDrive();
        wall.activate();
        turnArmDownFull();
        toggleClaw();
        double t = elapsedSecs();
        calculateDrivePowers(0, -0.3, 0);
        sendDrivePowers();
        while ( wall.h < 90 &&  elapsedSecs() - t < 2.0) { // TODO
            calculateDrivePowers(0, -0.3, getGyroPIDValue());
            sendDrivePowers();
        }
        stopDrive();
        toggleClaw();
        wait(0.9);
        if (finalArmPos.equals("up")) {
            turnArmUpFull();
        } else {
            turnArmDownDrag();
        }
    }

    // Calculate and set correct launch angle from anywhere on the field
    public void setAssistedLaunchAngle() {
        powerLauncher.setLaunchAngle(tower.findLaunchAngle(gyro.getAngle()));
        if (powerLauncher.launchAngle > powerLauncher.PERFECT_LAUNCH_ANGLE + 0.050) {
            powerLauncher.setLaunchAngle(powerLauncher.PERFECT_LAUNCH_ANGLE + 0.050);
        }
        if (powerLauncher.launchAngle < powerLauncher.PERFECT_LAUNCH_ANGLE - 0.060) {
            powerLauncher.setLaunchAngle(powerLauncher.PERFECT_LAUNCH_ANGLE - 0.060);
        }
    }

    // Rotate in place using the tower goal coordinates
    public void rotateWithCV(int offSet) { // TODO : OPTIMIZE
        targetGyroAngle = getReasonableGyroAngle(0);
        if (getAbsoluteGyroError() > 10) {
            rotateToPos(0, 0.4);
        }

        double error = tower.getLeftRightError(offSet - 5);
        CVDetectionPipeline.sleepTimeMS = 0;

        while (error > -10) { // TODO : WAS -25
            CVDetectionPipeline.sleepTimeMS = 0;
            error = tower.getLeftRightError(offSet - 5);
            calculateDrivePowers(0, 0, 0.2);
            sendDrivePowers();
        }
        stopDrive();
        wait(0.1);
        error = tower.getLeftRightError(offSet - 5);

        while (error < 0) {
            CVDetectionPipeline.sleepTimeMS = 0;
            error = tower.getLeftRightError(offSet - 5);
            calculateDrivePowers(0, 0, -0.15);
            sendDrivePowers();
        }
        stopDrive();
    }

    public int rotateWithCVInPhases(int phase, int offSet) {

        // Distance between tower goal and left side of screen: tower.x
        // Distance between tower goal and right side of screen: 320 - tower.x - tower.w
        double error = tower.getLeftRightError(offSet);

        if (phase >= 10) {
            activateFieldLocalization();
            targetGyroAngle = getReasonableGyroAngle(0);
            phase = 9;
        }

        switch (phase) {
            case 9:
                if (cameras.isStreaming() && cameras.isWebcamReady()) {
                    phase--;
                }
                break;
            case 8:
                if (getAbsoluteGyroError() < 15 && tower.isIdentified()) {
                    phase--;
                } else {
                    adjustAngle();
                }
                break;
            case 7:
                if (tower.getLeftRightError(offSet - 5) > -25) {
                    stopDrive();
                    resetPhaseTimeStamp();
                    phase--;
                } else {
                    CVDetectionPipeline.sleepTimeMS = 0;
                    calculateDrivePowers(0, 0, 0.2);
                    sendDrivePowers();
                }
                break;
            case 6:
                if (getPhaseTimePassed() > 0.1) {
                    phase--;
                }
                break;
            case 5:
                if (tower.getLeftRightError(offSet - 5) < 0) {
                    stopDrive();
                    phase = 1;
                } else {
                    CVDetectionPipeline.sleepTimeMS = 0;
                    calculateDrivePowers(0, 0, -0.15);
                    sendDrivePowers();
                }
                break;
            case 1:
                phase = powerLauncher.handleQueue(1);
                break;
        }

        return phase;
    }

    // Go to MID_POWERSHOT_POS before calling this
    public void shootPowerShots() {

        // Setup
        activateFieldLocalization();
        targetGyroAngle = getReasonableGyroAngle(0);

        setAssistedLaunchAngle();
        powerLauncher.changeLaunchAngle(-0.015);
        rotateWithCV(-148);

        powerLauncher.toggleOn(0.94);
        wait(0.4);
        indexRings(1);
        powerLauncher.toggleOff();

        rotateWithCV(-104);
        powerLauncher.toggleOn(0.94);
        wait(0.4);
        indexRings(1);
        powerLauncher.toggleOff();

        rotateWithCV(-66);
        powerLauncher.toggleOn(0.94);
        wait(0.4);
        indexRings(1);
        powerLauncher.toggleOff();
    }

    public void shootPowerShotsAngled() {
        // Setup
        activateFieldLocalization();
        targetGyroAngle = getReasonableGyroAngle(0);

        powerLauncher.setLaunchAnglePerfect();
        powerLauncher.toggleOn(0.85);
        rotateWithCV(-80);
        powerLauncher.setIndexerForwardPos();
        wait(0.2);
        powerLauncher.setIndexerBackPos();

        calculateDrivePowers(0, 0.6, 0, true);
        sendDrivePowers();

        powerLauncher.toggleOn(0.87);
        wait(0.24);
        powerLauncher.setIndexerForwardPos();
        wait(0.2);
        powerLauncher.setIndexerBackPos();

        powerLauncher.toggleOn(0.89);
        wait(0.24);
        powerLauncher.setIndexerForwardPos();
        wait(0.3);
        powerLauncher.setIndexerBackPos();
        wait(0.5);

        powerLauncher.toggleOff();
        stopDrive();
    }

    public void shootPowerShotsStrafe() {
        // Setup
        activateFieldLocalization();
        targetGyroAngle = getReasonableGyroAngle(0);

        powerLauncher.setLaunchAnglePerfect();
//        powerLauncher.changeLaunchAngle(-0.020);
        powerLauncher.toggleOn(0.85);
        rotateWithCV(-60);
        powerLauncher.setIndexerForwardPos();
        wait(0.2);
        powerLauncher.setIndexerBackPos();

        calculateDrivePowers(0, 0.6, 0, true);
        sendDrivePowers();


        powerLauncher.toggleOn(0.92);
        wait(0.2);
        powerLauncher.setIndexerForwardPos();
        wait(0.25);
        powerLauncher.setIndexerBackPos();

        powerLauncher.toggleOn(0.94);
        wait(0.28);
        powerLauncher.setIndexerForwardPos();
        wait(0.25);
        powerLauncher.setIndexerBackPos();
        wait(0.5);


        powerLauncher.toggleOff();
        stopDrive();
    }

    public void shootPowerShotsStrafeCV() {
        // Setup
        activateFieldLocalization();
        targetGyroAngle = getReasonableGyroAngle(0);

        powerLauncher.setLaunchAnglePerfect();
        powerLauncher.toggleOn(0.84);
        rotateWithCV(-80);

        powerLauncher.setIndexerForwardPos();
        wait(0.25);
        powerLauncher.setIndexerBackPos();
        movePowerShot();
        wait(0.25);

        powerLauncher.toggleOn(0.85);

        while (tower.getLeftRightError(-104) > 15) {
            CVDetectionPipeline.sleepTimeMS = 0;
            movePowerShot();
        }

        powerLauncher.setIndexerForwardPos();
        wait(0.25);
        powerLauncher.setIndexerBackPos();
        movePowerShot();
        wait(0.25);

        resetPhaseTimeStamp();
        powerLauncher.toggleOn(0.86);

        while (tower.getLeftRightError(-148) > 15) {
            CVDetectionPipeline.sleepTimeMS = 0;
            movePowerShot();
        }

        powerLauncher.setIndexerForwardPos();
        wait(0.5);
        powerLauncher.setIndexerBackPos();

        powerLauncher.toggleOff();
        stopDrive();
    }

    private void movePowerShot() {
        CVDetectionPipeline.sleepTimeMS = 0;
        calculateDrivePowers(0, 0.4, 0, true);
        sendDrivePowers();
    }

    // Strafe left and right until aligned with wobble goal
    public void alignToWobble() {
        while (tower.x > 130) {
            calculateDrivePowers(0.3, 0, getGyroPIDValue());
            sendDrivePowers();
        }
        stopDrive();
    }
}

// Documentation: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html