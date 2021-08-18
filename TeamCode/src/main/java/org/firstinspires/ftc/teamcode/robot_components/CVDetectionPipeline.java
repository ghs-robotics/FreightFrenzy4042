package org.firstinspires.ftc.teamcode.robot_components;

import org.firstinspires.ftc.teamcode.cv_objects.CVObject;
import org.firstinspires.ftc.teamcode.data.HSVConstants;
import org.firstinspires.ftc.teamcode.data.MyScalar;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ConcurrentModificationException;
import java.util.concurrent.CopyOnWriteArrayList;

import static java.lang.Thread.sleep;

// Pipeline for image processing
public class CVDetectionPipeline extends OpenCvPipeline implements HSVConstants {

    // The amount of time between image processing should take
    // The higher this value, the lower the lag during teleOp
    // Generally, letting this value be 0 during CV functions and 500 otherwise works rather well
    public static long sleepTimeMS = 0;

    // Stores the most recent input frame (so we can access it from CVObjects)
    public Mat currentMat = new Mat();

    // Auxiliary Mat objects for temporarily storing data
    private static Mat dst1 = new Mat();

    public CopyOnWriteArrayList<CVObject> activeObjects; // list of objects to be updated continuously

    // For sampling HSV values of individual pixels
    public String crosshairHSV = "";

    // This method is called (once?) in the background (not by us)
    @Override
    public void init(Mat firstFrame) {}

    // Processes image before displaying on phone screen
    // This method is called in the background (not by us) every time the camera receives a new
    // input, which happens multiple times a second while we're streaming
    @Override
    public Mat processFrame(Mat input) throws ConcurrentModificationException {

        // Resizes image to make processing more uniform
        Imgproc.resize(input, input, new Size(320, 240));

        // Creates Gaussian blur on image; makes things easier to detect
        Imgproc.GaussianBlur(input, input, new Size(5, 5), 80, 80);

        currentMat = input; // store the current frame in this class

        // Iterate through all the objects that are being analyzed continuously
        for (CVObject obj : activeObjects) {
            obj.updateData();
        }

        // Updates crosshairValue of center point
//        /*
        Imgproc.cvtColor(input, dst1, Imgproc.COLOR_BGR2HSV);
        crosshairHSV = findHSV(dst1, input.rows()/2, input.cols()/2).toString();
        Imgproc.rectangle(
                input,
                new Point(input.cols()/2 - 5, input.rows()/2 - 5),
                new Point(input.cols()/2 + 5, input.rows()/2 + 5),
                GREEN_BGR,
                1);
//         */

        // Draw some rectangles
//        Imgproc.rectangle(currentMat, new Point(0, 0), new Point(320, 40), GREEN_BGR, -1);
//        Imgproc.rectangle(currentMat, new Point(0, 60), new Point(320, 240), GREEN_BGR, -1);

        // If sleepTimeMS > 0, the program will wait, thereby decreasing FPS and reducing lag during TeleOp
        try {
            sleep(sleepTimeMS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return currentMat;
    }

    // Finds HSV values of a point
    public static MyScalar findHSV(Mat input, int row, int col) {
        double[] val = input.get(row, col); // double[] array with HSV values
        return new MyScalar((int) val[0], (int) val[1], (int) val[2]);
    }
}