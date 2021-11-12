package org.firstinspires.ftc.teamcode.robot_components.navigation;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.data.Vector2;



import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.data.Vector2;

import java.util.Vector;

public class DeadReckoning {
        public DcMotor deadWheelX;

        final double INITIAL_X_POSITION = 0;
        final int INITIAL_X_TICKS = getRawEncoderValues();
        final int TICKS_PER_REVOLUTION = 8192;
        final double WHEEL_DIAMETER_MM = 35.0; // not radius!
        final double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * Math.PI;

        private double deadWheelXRaw;

        private int prevX;

        public DeadReckoning(DcMotor x) {
            this.deadWheelX = x;
            this.prevX = 0;
        }

        //use the gyroscope for getting angle

        //TODO parse dead wheel inputs

        /**
         * get both raw encoder values as int[2].
         * @return array containing {x, y} encoder ticks.
         */
        public int getRawEncoderValues() {
            return deadWheelX.getCurrentPosition();
        }

        /**
         * get the distance travelled since this function was last called
         */
        public Vector calculateDistanceChange() {
            int currentX = deadWheelX.getCurrentPosition();
            // average angles
            double xChange = encoderTicksToDistance(currentX - prevX);
            return new Vector((int) xChange);
        }
        //Calculates X position based on odometry pod encoder tick values
        public double calculatePosition() {
            //Finds the change in x position in cm, based on odometry ticks
            int deltaTicks = getRawEncoderValues() - INITIAL_X_TICKS;
            double currentXDistance = 10 * encoderTicksToDistance(deltaTicks);
            double currentXPos = currentXDistance + INITIAL_X_POSITION;
            return currentXPos;
        }

        /**
         * Convert encoder ticks to physical linear distance.
         * Uses TICKS_PER_REVOLUTION and WHEEL_CIRCUMFERENCE_MM.
         * @param ticks the number of encoder ticks.
         * @return The linear distance in mm.
         */
        public double encoderTicksToDistance(int ticks) {
            double revolutions = ((double)ticks)/TICKS_PER_REVOLUTION;
            return revolutions * WHEEL_CIRCUMFERENCE_MM;
        }

}
