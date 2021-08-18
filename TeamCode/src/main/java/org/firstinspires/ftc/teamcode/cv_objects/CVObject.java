package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.data.HSVConstants;
import org.firstinspires.ftc.teamcode.data.MyScalar;
import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

// Generic object that can be detected using OpenCV
public abstract class CVObject implements HSVConstants {

    public String name = "unspecified";
    protected boolean identified = false;
    protected boolean active = false; // true if pipeline is actively updating this object's data

    public CVDetectionPipeline pipeline;

    // Contains frame that will be used for object detection
    // constantly updated (in updateData)
    protected Mat currentHSVMat;
    protected Mat currentHSVMask; // Stores the mask
    protected Mat hierarchy; // Auxiliary
    protected Mat kernel; // Auxiliary

    protected Scalar lowerHSV;
    protected Scalar upperHSV;

    // All measured in pixels (on the screen)
    public int x = 0;
    public int y = 0;
    public int w = 0; // width
    public int h = 0; // height

    // Values to be targeted
    public int targetX = 0;
    public int targetY = 0;
    public int targetW = 0;
    public int targetH = 0;

    // Amount of screen covered by a white rectangle during image processing; ranges from 0 to 1
    public double cover = 0;

    // PIDs for x and y position of robot
    public PIDController breadthPID; // Controls side to side motion of robot
    public PIDController depthPID; // Controls forward and backward motion of the robot

    // CVObject constructor
    // Used by its child class (through the keyword "super")
    public CVObject(String name, CVDetectionPipeline pipeline) {
        this.name = name;
        this.pipeline = pipeline;
        this.currentHSVMat = new Mat();
        this.currentHSVMask = new Mat();
        this.hierarchy = new Mat();
    }

    public CVObject(String name, CVDetectionPipeline pipeline, PIDController breadthPID, PIDController depthPID) {
        this(name, pipeline);
        this.breadthPID = breadthPID;
        this.depthPID = depthPID;
    }

    // Add to list of objects that are continually updated in the pipeline
    // Objects should be active before they can be used in a meaningful way
    public void activate() {
        if (!pipeline.activeObjects.contains(this)) {
            pipeline.activeObjects.add(this);
            active = true;
        }
    }

    // Creates solid rectangles to cover up background noise
    protected void coverBackground() {
        Imgproc.rectangle(
                currentHSVMat,
                new Point(0, 0),
                new Point(SCREEN_WIDTH, (int) (cover * SCREEN_HEIGHT)),
                GREEN_BGR, -1
        );
    }

    // Remove from list of objects that are continually updated in the pipeline
    public void deactivate() {
        if (pipeline.activeObjects.contains(this)) {
            pipeline.activeObjects.remove(this);
            active = false;
        }
    }

    // Finds HSV values of a point
    protected MyScalar findHSV(int row, int col) {
        double[] val = currentHSVMat.get(row, col); // double[] array with HSV values
        return new MyScalar((int) val[0], (int) val[1], (int) val[2]);
    }

    public int getAbsErrorX() {
        return Math.abs(targetX - x);
    }

    public int getAbsErrorY() {
        return Math.abs(targetY - y);
    }

    public int getAbsErrorW() {
        return Math.abs(targetW - w);
    }

    public int getAbsErrorH() {
        return Math.abs(targetH - h);
    }

    public int getErrorX() {
        return targetX - x;
    }

    public int getErrorY() {
        return targetY - y;
    }

    public int getErrorW() {
        return targetW - w;
    }

    public int getErrorH() {
        return targetH - h;
    }

    // Uses x value by default
    public double getBreadthPIDValue() {
        if (identified) {
            return -breadthPID.calcVal(getErrorX());
        }
        else {
            return 0;
        }
    }

    // Uses width by default
    public double getDepthPIDValue() {
        if (identified) {
            return depthPID.calcVal(getErrorW());
        }
        else {
            return 0;
        }
    }

    // True if val is between min and max
    public boolean inRange(double val, double min, double max) {
        return (min <= val && val <= max);
    }

    public boolean isActive() {
        return active;
    }

    public boolean isIdentified() {
        return identified;
    }

    // A test to verify the approximate size of an object
    // Should be different for each child class
    abstract boolean isReasonable(int x, int y, int w, int h);

    // Should be overridden in subclasses having fewer than two PIDs
    public void resetPIDs() {
        breadthPID.resetValues();
        depthPID.resetValues();
    }

    public void setTargetX(int targetX) {
        this.targetX = targetX;
        resetPIDs();
    }

    public void setTargetXW(int[] vals) {
        this.targetX = vals[0];
        this.targetW = vals[1];
        resetPIDs();
    }

    public void setTargetY(int targetY) {
        this.targetY = targetY;
        resetPIDs();
    }

    public void setTargetW(int targetW) {
        this.targetW = targetW;
        resetPIDs();
    }

    public void setTargetH(int targetH) {
        this.targetH = targetH;
        resetPIDs();
    }

    public void setToNotIdentified() {
        this.x = 0;
        this.y = 0;
        this.w = 0;
        this.h = 0;
        identified = false;
    }

    // This displays a bunch of useful information for telemetry
    @Override
    public String toString() {
        String s = name.toUpperCase() + " (x = " + x + ", y = " + y + ", w = " + w + ", h = " + h + ")";
        if (!active) {
            s = "NOT ACTIVE: " + s;
        }
        else if (!identified) {
            s = "NOT IDENTIFIED: " + s;
        }
        return s;
    }

    // Updates coordinates and identified boolean
    // ONLY CALL THIS FROM WITHIN THE PIPELINE
    // This method is constantly called by the object's pipeline as long as it is active
    public void updateData() {
        Mat input = pipeline.currentMat; // Edits to this Mat will display on the phone screen

        // Converts color from BGR (default format for OpenCV) to HSV (easier format to process with)
        Imgproc.cvtColor(input, currentHSVMat, Imgproc.COLOR_BGR2HSV);

        // Adds rectangles
        coverBackground();

        // Filters colors within certain color range
        Core.inRange(currentHSVMat, this.lowerHSV, this.upperHSV, currentHSVMask);

        // Finds the contours of the object and stores them in an ArrayList
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(currentHSVMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw contours on the src image
//        Imgproc.drawContours(input, contours, -1, GREEN_BGR, 1, Imgproc.LINE_8, hierarchy, 2, new Point());

        // Creates a rectangle called rect with default value of 0 for x, y, width, and height
        Rect largestRect = new Rect();

        // Iterates through all of the contours and finds the largest bounding rectangle
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (largestRect.area() < rect.area() && isReasonable(rect.x, rect.y, rect.width, rect.height)) {
                largestRect = rect;
            }
        }

        // Draws largest rect on src image
        Imgproc.rectangle(input, largestRect, GREEN_BGR, 2);

        // Updates coordinates
        this.x = largestRect.x;
        this.y = largestRect.y;
        this.w = largestRect.width;
        this.h = largestRect.height;

        // If x, y, w, h are all zero, the object is NOT being identified
        identified = (x + y + w + h > 0);
    }
}
