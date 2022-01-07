package org.firstinspires.ftc.teamcode.robot_components.navigation;

import android.os.DropBoxManager;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.data.Vector2;

public class OdometryModule {

    public DcMotor deadWheelX;
    public DcMotor deadWheelY;
    private Gyro gyro;

    final int TICKS_PER_REVOLUTION = 8192;
    final double WHEEL_DIAMETER_MM = 35.0; // not radius!
    final double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * Math.PI;

    private double deadWheelXRaw;
    private double deadWheelYRaw;

    private int prevX;
    private int prevY;
    private double prevAngle;
    Telemetry telemetry;
    public boolean hasXMotor;
    public boolean hasYMotor;

    public int startingTicksX;
    public int startingTicksY;

    public OdometryModule(DcMotor y, Gyro g) {
        this.hasXMotor = false;
        this.hasYMotor = true;
        this.deadWheelY = y;
        this.prevX = this.prevY = 0;
        this.prevAngle = 0.0;
        this.gyro = g;
        this.startingTicksY = y.getCurrentPosition();
    }

    public OdometryModule(DcMotor x, DcMotor y, Gyro g) {
        this.hasXMotor = true;
        this.hasYMotor = true;
        this.deadWheelX = x;
        this.deadWheelY = y;
        this.prevX = this.prevY = 0;
        this.prevAngle = 0.0;
        this.gyro = g;
        this.startingTicksX = x.getCurrentPosition();
        this.startingTicksY = y.getCurrentPosition();
    }

    //use the gyroscope for getting angle

    //TODO parse dead wheel inputs

    /**
     * get both raw encoder values as int[2].
     * @return array containing {x, y} encoder ticks.
     */
    public int[] getRawEncoderValues() {
        return new int[] {hasXMotor ? deadWheelX.getCurrentPosition() - startingTicksX : 0,
                hasYMotor ? deadWheelY.getCurrentPosition() - startingTicksY : 0};
    }

    public int[] getMillimeterDist() {
        return new int[]{hasXMotor ? (int)encoderTicksToDistance(deadWheelX.getCurrentPosition() - startingTicksX) : 0,
                hasYMotor ? (int)encoderTicksToDistance(deadWheelY.getCurrentPosition() - startingTicksY) : 0};
    }

    /**
     * get the distance travelled since this function was last called
     */
    public Vector2 calculateDistanceChange() {
        int currentX = deadWheelX.getCurrentPosition();
        int currentY = deadWheelY.getCurrentPosition();
        double currentAngle = gyro.getAngle();
        // average angles
        double avgAngle = (currentAngle + this.prevAngle)/2.0;
        double xChange = Math.cos(avgAngle) * encoderTicksToDistance(currentX - prevX);
        double yChange = Math.sin(avgAngle) * encoderTicksToDistance(currentY - prevY);
        return new Vector2(xChange, yChange);
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
