package org.firstinspires.ftc.teamcode.cv_objects;

import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.objdetect.CascadeClassifier;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVPipeline extends OpenCvPipeline
{
    Mat IntegralImage = new Mat();

    private CascadeClassifier faceClassifier = new CascadeClassifier(
            "mtp://REV_Robotics_Control_Hub_v1.0_809a3b987eebdaa7/Internal%20shared%20storage/FIRST/CascadeFiles/FaceCascade");
    private MatOfRect faces = new MatOfRect();

    private CascadeClassifier boxClassifier = new CascadeClassifier(
            "mtp://REV_Robotics_Control_Hub_v1.0_809a3b987eebdaa7/Internal%20shared%20storage/FIRST/CascadeFiles/CubeCascade");
    private MatOfRect boxes = new MatOfRect();

    private CascadeClassifier wiffleClassifier = new CascadeClassifier(
            "mtp://REV_Robotics_Control_Hub_v1.0_809a3b987eebdaa7/Internal%20shared%20storage/FIRST/CascadeFiles/WiffleCascade");;
    private MatOfRect wiffles = new MatOfRect();

    private CascadeClassifier duckClassifier = new CascadeClassifier(
            "mtp://REV_Robotics_Control_Hub_v1.0_809a3b987eebdaa7/Internal%20shared%20storage/FIRST/CascadeFiles/DuckCascade");;
    private MatOfRect ducks = new MatOfRect();

    @Override
    public Mat processFrame(Mat input) {
        faceClassifier.detectMultiScale(input, faces);
        boxClassifier.detectMultiScale(input, boxes);
        wiffleClassifier.detectMultiScale(input, wiffles);
        duckClassifier.detectMultiScale(input, ducks);
        return null;
    }

    public MatOfRect returnResultsFaces() {return faces;}

    public MatOfRect returnResultsBoxes(){
        return boxes;
    }

    public MatOfRect returnResultsWiffles(){ return wiffles; }

    public MatOfRect returnResultsDucks(){ return ducks; }
}