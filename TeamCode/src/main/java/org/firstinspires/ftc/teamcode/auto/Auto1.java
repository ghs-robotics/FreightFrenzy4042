package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.cv.CVModule;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.robot_components.cv.CameraManager;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;


@Autonomous
public class Auto1 extends LinearOpMode implements FieldPositions {

    // Declare OpMode members
    Robot robot;
    CameraManager cameraManager; //320 by 240 (x by y)

    int detectBarcode() {
        int shippingX = 0;
        //Magically assign shippingX here
        return shippingX / 107;
    }
    private CVModule robot;
    OpenCvWebcam webcam;

    CVPipeline pipeline;

    @Override
    public void runOpMode()
    {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        final OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        pipeline= new CVPipeline();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                waitForStart();
                while(opModeIsActive()){
                    camera.setPipeline(pipeline);
                    camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                    // Usually this is where you'll want to start streaming from the camera (see section 4)
                }
            }
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Boxes", pipeline.returnResultsBoxes());
            //telemetry.addData("Wiffles", pipeline.returnResultsWiffles());
            //telemetry.addData("Ducks", pipeline.returnResultsDucks());
            telemetry.update();
            sleep(100);
        }
    }
}
//top - spawn point closer to the warehouse
//bottom spawn point closer to the carousal



//Auto mode for prioritizing freight delivery - top

//Detect the barcode and place box to the corresponding level
    int barcode = detectBarcode(); //0 = left, 1 = middle, 2 = right
    robot.raiseDeployer(100 * barcode); //100 is a placeholder for 1 level up the shipping hub//
            robot.dumpElement();
//Move back to spawn with the intake face the warehouse
        //Move back to spawn with the intake facing the warehouse
        robot.turn(90); //Face the warehouse (assuming 90 means right)
        robot.move(); //Move 100 units in the 90 degree direction

//Detect the weight of the freight if possible

//Deliver the freight to the team shipping hub's bottom level

//Repeat the last three steps until there's 2-5 secs left on the auto timer

        //Park in the warehouse
        Robot.park()




//Auto mode for prioritizing0 freight delivery - bottom

//Detect the barcode and place box to the corresponding level

//Move around our teammate's robot with the intake face the warehouse

//Detect the weight of the freight if possible

//Deliver the freight to the team shipping hub's bottom level

//Repeat the last two steps until there's 2-5 secs left on the auto timer

//Park in the warehouse





        //Auto mode for teammate prioritizing freight delivery and no anti-collision system

        //Detect the barcode and place box to the corresponding level

        //If teammate places down their box, navigate around them, pick it up and deliver to corresponding shipping hub level

//Park in warehouse out of our teammate's way






//Auto mode for teammate with bad auto - top

//Detect the barcode and place the box to corresponding level

//Move around the teammate's robot to knock off the duck

//Move back to spawn with the side with the intake facing the warehouse

//Detect the weight of the freight if possible

//Deliver the freight to the team shipping hub's bottom level

//Repeat the last two steps until there's 2-5 secs left on the auto timer

//Park in the warehouse





//Auto mode for teammate with bad auto - bottom

//Detect the barcode

//Move to the carousal and knock off the duck

//Deliver the box to the shipping hub

//Move around the other robot with intake facing warehouse

//Detect the weight of the freight if possible

//Deliver the freight to the team shipping hub's bottom level

//Check if the shipping hub is balanced

//Repeat the last three steps until there's 2-5 secs left on the auto timer

//Park in the warehouse





//Auto mode for prioritizing carousal and possibly balancing the shared hub

//Detect the barcode and place game element to corresponding level

//Move to the carousal to knock off the duck

//Stay at the shared shipping hub and try to balance the hub if it gives points, if not go do freight

//2-5 seconds before autonomous ends go park in the warehouse



        ///////////////////////////////////////////////////////////////////////////////////////
        //                   If we can't park both robots in the warehouse                   //
        ///////////////////////////////////////////////////////////////////////////////////////




//Auto mode for prioritizing freight delivery - top

//Detect the barcode and place box to the corresponding level

//Move back to spawn with the intake face the warehouse

//Detect the weight of the freight if possible

//Deliver the freight to the team shipping hub's bottom level

//Repeat the last three steps until there's 2-5 secs left on the auto timer

//Park in the alliance shipping hub





//Auto mode for prioritizing0 freight delivery - bottom

//Detect the barcode and place box to the corresponding level

//Move around our teammate's robot with the intake face the warehouse

//Detect the weight of the freight if possible

//Deliver the freight to the team shipping hub's bottom level

//Repeat the last two steps until there's 2-5 secs left on the auto timer

//Park in the alliance shipping hub





//Auto mode for teammate prioritizing freight delivery and no anti-collision system

//Detect the barcode and place box to the corresponding level

//If teammate places down their box, navigate around them, pick it up and deliver to corresponding shipping hub level

//Park in alliance shipping hub out of our teammate's way






//Auto mode for teammate with bad auto - top

//Detect the barcode and place the box to corresponding level

//Move around the teammate's robot to knock off the duck

//Move back to spawn with the side with the intake facing the warehouse

//Detect the weight of the freight if possible

//Deliver the freight to the team shipping hub's bottom level

//Repeat the last two steps until there's 2-5 secs left on the auto timer

//Park in the alliance shipping hub





//Auto mode for teammate with bad auto - bottom

//Detect the barcode

//Move to the carousal and knock off the duck

//Deliver the box to the shipping hub

//Move around the other robot with intake facing warehouse

//Detect the weight of the freight if possible

//Deliver the freight to the team shipping hub's bottom level

//Check if the shipping hub is balanced

//Repeat the last three steps until there's 2-5 secs left on the auto timer

//Park in the alliance shipping hub





//Auto mode for prioritizing carousal and possibly balancing the shared hub

//Detect the barcode and place game element to corresponding level

//Move to the carousal to knock off the duck

//Stay at the shared shipping hub and try to balance the hub if it gives points, if not go do freight

//2-5 seconds before autonomous ends go park in the alliance shipping hub
