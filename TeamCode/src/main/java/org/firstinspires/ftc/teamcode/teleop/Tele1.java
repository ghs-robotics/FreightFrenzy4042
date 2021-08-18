/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.Controller;
import org.firstinspires.ftc.teamcode.robot_components.CVRobot;

@TeleOp(name="Tele1", group="Linear Opmode")
public class Tele1 extends LinearOpMode implements FieldPositions {
    
    // Declare OpMode members
    CVRobot robot;
    Controller controller1;
    Controller controller2;

    int queue;  // Keeps track of how many rings are "in line" to be shot
    int movePhase; // 0 is normal; not 0 means robot will perform an automated function
    int autoAimPhase; // 0 is normal; not 0 means robot will perform an automated function
    int offSet; // offset from the tower goal for automated rotation

    double timeStamp;

    String intakeSetting; // "normal," "in," "out"

    @Override
    public void runOpMode() {

        // Code to run ONCE when the driver hits INIT
        robot = new CVRobot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1); // Whoever presses start + a
        controller2 = new Controller(gamepad2); // Whoever presses start + b

        queue = 0;
        movePhase = 0;
        autoAimPhase = 0;
        offSet = 14;
        timeStamp = 0;
        intakeSetting = "normal";

        robot.initWithCV();
        robot.powerLauncher.setLaunchAngleLoading();
        robot.turnArmUpFull();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.cameras.webcam.pauseViewport();
        robot.activateFieldLocalization();
//        telemetry.setMsTransmissionInterval(20);
        robot.wobble.activate();

        robot.resetGyroAngle();
        robot.resetElapsedTime();

        while (opModeIsActive()) {

            // Registers controller input
            controller1.update();
            controller2.update();



            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 1 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------

            // NOTE: TO USE THESE FUNCTIONS PRESS START A

            applyController1Joysticks();

            // Toggle intake on/off
            if (controller1.a.equals("pressing")) {
                if (intakeSetting.equals("in")) {
                    intakeSetting = "normal";
                } else {
                    intakeSetting = "in";
                }
            }

            // Toggle intake backward/off
            if (controller1.b.equals("pressing")) {
                if (intakeSetting.equals("out")) {
                    intakeSetting = "normal";
                } else {
                    intakeSetting = "out";
                }
            }

            // Auto aim and shoot
            if (controller1.x.equals("pressing")) {
                intakeSetting = "normal";
                robot.powerLauncher.toggleOn();
                autoAimPhase = 15;
            }

            // Terminate any automated functions and stop streaming
            if (controller1.y.equals("pressing")) {
                movePhase = 0;
                autoAimPhase = 0;
                robot.turnWhiskerUp();
                robot.powerLauncher.toggleOff();
            }

            // Reset gyro in case of emergency
            if (controller1.left_bumper.equals("pressing")) {
                robot.resetGyroAngle();
            }

            // Reset any controls
            if (controller1.left_stick_button.equals("pressing")) {
                robot.speed = 1;
                movePhase = 0;
                autoAimPhase = 0;
                CVDetectionPipeline.sleepTimeMS = 500;
                robot.powerLauncher.toggleOff();
            }

            if (controller1.dpad_right.equals("pressed")) {
                robot.tower.setTargetXW(LEFT_POWERSHOT_POS);
                movePhase = 4;
            }
            else if (controller1.dpad_up.equals("pressed")) {
                movePhase = 0;
                autoAimPhase = 0;
                robot.shootPowerShotsStrafeCV();
            }
            else if (controller1.dpad_left.equals("pressed")) {
                robot.tower.setTargetXW(PERFECT_LAUNCH_POS);
                movePhase = 4;
            }
            else if (controller1.dpad_down.equals("pressed")) {
                robot.cameras.stopStreaming();
                movePhase = 0;
                autoAimPhase = 0;
            }



            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 2 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------

            // NOTE: TO USE THESE FUNCTIONS, PRESS START B


            // Checks if any rings need to be shot and takes care of indexing
            if (queue > 0) {
                queue = robot.powerLauncher.handleIndexQueue(queue);
                if (queue == 0) {
                    robot.powerLauncher.toggleOff(); // toggle launcher off after indexing
                }
            }

            // Intake stuff
            if (intakeSetting.equals("in")) {
                robot.powerLauncher.setLaunchAngleLoading();
                robot.runIntake(0.9);
            }
            else if (intakeSetting.equals("out")) {
                robot.powerLauncher.setLaunchAngleLoading();
                robot.runIntake(-0.9);
            }
            else {
                // Run intake with right joystick TODO
                if (controller2.right_stick_y > 0) {
                    robot.runIntake(-0.9 * controller2.right_stick_y);
                }
                else {
                    robot.runIntake((controller2.right_stick_button.equals("pressed") ? -0.9 : -0.6) * controller2.right_stick_y);
                }
            }

            // Set loading launch angle if the intake is running
            if (controller2.right_stick_y != 0) {
                robot.powerLauncher.setLaunchAngleLoading();
                intakeSetting = "normal";
            }

            // Toggle intake to gather rings
            if (controller2.left_trigger + controller2.right_trigger > 1.8) {
                intakeSetting = "in";
            }

            // Right bumper toggles the claw
            if (controller2.right_bumper.equals("pressing")) {
                robot.toggleClaw();
            }

            // Left bumper turns arm
            if (controller2.left_bumper.equals("pressing")) {
                robot.turnArm();
            }

            // Index 3 rings
            if (controller2.a.equals("pressing")) {
//                robot.powerLauncher.resetQueueTimeStamp();
                queue = 10;
            }

            // Toggle launcher and set perfect launch angle
            // Note: You can interrupt queues this way!
            if (controller2.b.equals("pressing")) {
                robot.powerLauncher.setLaunchAnglePerfect();
                robot.powerLauncher.toggle();
                queue = 0;
            }

            // Manually index one ring (e.g. for powershots)
            if (controller2.x.equals("pressing")) {
                robot.indexRings(1);
            }

            // Toggle launcher motors
            if (controller2.y.equals("pressing")) {
                robot.powerLauncher.toggle();
                queue = 0;
            }

            //
            if (controller2.left_trigger > 0.7) {
                robot.turnWhiskerUp();
            }

            //
            if (controller2.right_trigger > 0.7) {
                robot.turnWhiskerDown();
            }

            // Angle the launcher down a tiny bit
            if (controller2.dpad_up.equals("pressing")) {
                robot.powerLauncher.PERFECT_LAUNCH_ANGLE -= 0.005;
            }

            // Angle the launcher up a tiny bit
            if (controller2.dpad_down.equals("pressing")) {
                robot.powerLauncher.PERFECT_LAUNCH_ANGLE += 0.005;
            }

            // Set current angle to default angle
            if (controller2.dpad_right.equals("pressing")) {
                robot.powerLauncher.setCurrentAngleToDefault();
            }

            //
            if (controller2.dpad_left.equals("pressing")) {
                robot.setAssistedLaunchAngle();
            }

            // Change current launch Angle (but keep default the same)
            if (controller2.left_stick_y != 0) {
                double val = -((int) (40 * controller2.left_stick_y)) / 1000.0;
                robot.powerLauncher.changeLaunchAngleGradually(val);
            }

            // Go to default launch angle
            if (controller2.left_stick_button.equals("pressing")) {
                robot.powerLauncher.setLaunchAnglePerfect();
            }
        }
    }

    private void applyController1Joysticks() {

        // Regular driving
        if (movePhase == 0 && autoAimPhase == 0) {
            CVDetectionPipeline.sleepTimeMS = 500;
            runDriveFunctions(false);
        }

        // Stuff that happens during automated functions
        else {

            // Interrupting an automated function without terminating it
            if (controller1.joyStickApplied()) {
                runDriveFunctions(true);
            }

            // Checks if the robot should be performing an automated move function
            else if (movePhase > 0) {
                movePhase = robot.moveInPhases(movePhase);
            }

            // Checks if the robot should be performing an automated shoot function
            else if (autoAimPhase > 0 && controller2.right_stick_y == 0) {
                autoAimPhase = robot.autoShootInPhases(autoAimPhase);
                robot.setAssistedLaunchAngle();
            }

            // Makes sure the launcher gets powered off TODO
            if (autoAimPhase == 1 || autoAimPhase == 2) {
                autoAimPhase = robot.autoShootInPhases(autoAimPhase);
                CVDetectionPipeline.sleepTimeMS = 500;
            } else {
                CVDetectionPipeline.sleepTimeMS = 0;
            }
        }

        robot.updateDrive(); // Also updates telemetry
    }

    private void runDriveFunctions(boolean autoTurret) {

        // Regular meta drive for mecanum wheel drivebase
        if (!controller1.right_bumper.equals("pressed")) {
            robot.calculateDrivePowers(
                    controller1.left_stick_x,
                    controller1.left_stick_y,
                    controller1.right_stick_x,
                    controller1.right_stick_y
            );
        }

        // Adjust angle if we're holding down the right bumper
        else if (autoTurret && robot.tower.isIdentified() && robot.tower.isActive()) {
            // Always rotate to face tower goal
            CVDetectionPipeline.sleepTimeMS = 0;
            if (controller2.right_stick_y == 0) {
                robot.setAssistedLaunchAngle();
            }
            robot.calculateDrivePowers(controller1.left_stick_x, controller1.left_stick_y, robot.tower.getRotPIDVal(14, 10), true);
        }
        else {
            robot.calculateDrivePowers(controller1.left_stick_x, controller1.left_stick_y, 1.0, 0);
        }

        // Micro-adjust robot's angle
        if (controller1.left_trigger > 0.1) {
            robot.calculateDrivePowers(0, 0, -0.35 * controller1.left_trigger);
        }
        else if (controller1.right_trigger > 0.1) {
            robot.calculateDrivePowers(0, 0, 0.35 * controller1.right_trigger);
        }
    }
}
