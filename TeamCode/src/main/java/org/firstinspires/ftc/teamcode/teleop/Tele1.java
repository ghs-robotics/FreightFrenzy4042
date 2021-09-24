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

import android.annotation.SuppressLint;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.media.Image;
import android.util.Size;

import androidx.annotation.NonNull;
import androidx.camera.core.Camera;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import androidx.camera.core.Preview;
import androidx.lifecycle.LifecycleOwner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot_components.input.Btn;
import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.input.Controller;
import org.firstinspires.ftc.teamcode.robot_components.cv.CVModule;

import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

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

            Thread initCam = new Thread() {
                public void run() {
                    ExecutorService executor = Executors.newSingleThreadExecutor();
                    ImageAnalysis imageAnalysis =
                            new ImageAnalysis.Builder()
                                    .setTargetResolution(new Size(720, 1280))
                                    .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                                    .build();

                    imageAnalysis.setAnalyzer(executor, new ImageAnalysis.Analyzer() {
                        public void analyze(ImageProxy imageProxy) {
                            //int rotationDegrees = imageProxy.getImageInfo().getRotationDegrees();
                            // insert your code here.
                            @SuppressLint("UnsafeExperimentalUsageError") Image image = imageProxy.getImage();
                            Bitmap tempMap = toBitmap(image);
                            Bitmap bitmap = Bitmap.createScaledBitmap(tempMap, 224, 224, true);
                            ByteBuffer input = ByteBuffer.allocateDirect(224 * 224 * 3 * 4).order(ByteOrder.nativeOrder());
                            for (int y = 0; y < 224; y++) {
                                for (int x = 0; x < 224; x++) {
                                    int px = bitmap.getPixel(x, y);

                                    // Get channel values from the pixel value.
                                    int r = Color.red(px);
                                    int g = Color.green(px);
                                    int b = Color.blue(px);

                                    // Normalize channel values to [-1.0, 1.0]. This requirement depends
                                    // on the model. For example, some models might require values to be
                                    // normalized to the range [0.0, 1.0] instead.
                                    float rf = (r - 127) / 255.0f;
                                    float gf = (g - 127) / 255.0f;
                                    float bf = (b - 127) / 255.0f;

                                    input.putFloat(rf);
                                    input.putFloat(gf);
                                    input.putFloat(bf);
                                }
                            }
                            int colour = tempMap.getPixel(0, 0);
                            int red = Color.red(colour);
                            int green = Color.green(colour);
                            int blue = Color.blue(colour);
                            int alpha = Color.alpha(colour);
                            System.out.println(red + ", " + blue + ", " + green + ", " + alpha);
                        };
                    });

                }
            };

            initCam.start();



            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 2 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // NOTE: TO USE THESE FUNCTIONS, PRESS START B
            //OPERATOR FUNCTIONS

        }
    }
    private Bitmap toBitmap(Image image) {
        Image.Plane[] planes = image.getPlanes();
        ByteBuffer yBuffer = planes[0].getBuffer();
        ByteBuffer uBuffer = planes[1].getBuffer();
        ByteBuffer vBuffer = planes[2].getBuffer();

        int ySize = yBuffer.remaining();
        int uSize = uBuffer.remaining();
        int vSize = vBuffer.remaining();

        byte[] nv21 = new byte[ySize + uSize + vSize];
        //U and V are swapped
        yBuffer.get(nv21, 0, ySize);
        vBuffer.get(nv21, ySize, vSize);
        uBuffer.get(nv21, ySize + vSize, uSize);

        YuvImage yuvImage = new YuvImage(nv21, ImageFormat.NV21, image.getWidth(), image.getHeight(), null);
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, yuvImage.getWidth(), yuvImage.getHeight()), 75, out);

        byte[] imageBytes = out.toByteArray();
        return BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
    }
}
