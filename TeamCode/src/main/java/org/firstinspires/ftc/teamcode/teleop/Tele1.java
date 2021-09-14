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

import org.firstinspires.ftc.teamcode.robot_components.input.Btn;
import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.input.Controller;
import org.firstinspires.ftc.teamcode.robot_components.cv.CVModule;

@TeleOp(name="Tele1", group="Linear Opmode")
public class Tele1 extends LinearOpMode implements FieldPositions {
    
    // Declare OpMode members
    CVModule robot;
    Controller controller1;
    Controller controller2;

    @Override
    public void runOpMode() {

        // Code to run ONCE when the driver hits INIT
        robot = new CVModule(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1); // Whoever presses start + a
        controller2 = new Controller(gamepad2); // Whoever presses start + b

        robot.initWithCV();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.cameras.webcam.pauseViewport();
        robot.activateFieldLocalization();

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
            //DRIVER FUNCTIONS
            if (controller1.right_bumper != Btn.PRESSED) {
                robot.calculateDrivePowers(
                        controller1.left_stick_x,
                        controller1.left_stick_y,
                        controller1.right_stick_x,
                        controller1.right_stick_y
                );
            }

            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 2 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // NOTE: TO USE THESE FUNCTIONS, PRESS START B
            //OPERATOR FUNCTIONS

        }
    }
}
