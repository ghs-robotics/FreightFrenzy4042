package org.firstinspires.ftc.teamcode.navigation.tasks;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv_objects.CVManager;
import org.firstinspires.ftc.teamcode.navigation.RobotPosition;
import org.firstinspires.ftc.teamcode.navigation.Task;
import org.firstinspires.ftc.teamcode.robot_components.robot.Robot;
import org.opencv.core.Rect;

import java.util.HashMap;
import java.util.List;

/**
 * Deposit freight into an the chosen goal (this includes extension, drop, and retraction)
 */
public class ScanCode implements Task {
    public double timesScanned;
    public boolean finishedScan;
    CVManager manager;
    static HashMap<Rect, Integer> faceLocations = new HashMap<>(); //Rect key and Integer value (# times seen)

    public ScanCode(HardwareMap hM, Telemetry t) {
        timesScanned = 0;
        finishedScan = false;
        manager = new CVManager(hM, t);
        manager.initCamera();
    }

    public void init() {}

    public boolean update(RobotPosition currentPosition, Robot robot) {
        List<Rect> faces = manager.getFaces();
        for (Rect face : faces) {
            boolean foundMatch = false;
            for (Rect loc : faceLocations.keySet()) {
                if (Math.abs(face.x - loc.x) < 5 && Math.abs(face.y - loc.y) < 5) {
                    faceLocations.put(loc, faceLocations.get(loc) + 1);
                    foundMatch = true;
                    break;
                }
            }
            if (!foundMatch) {
                faceLocations.put(face, 1);
            }
        }

        timesScanned++;
        if (timesScanned >= 30) {
            robot.setBarcodePos(mostFrequent(faceLocations));
            finishedScan = true;
        }
        return finishedScan;
    }

    public static HashMap<Rect, Integer> getMap() {
        return faceLocations;
    }

    public Rect mostFrequent(HashMap<Rect, Integer> input) {
        Integer highestFrequency = 0;
        Rect bestKey = new Rect();
        for (Rect key : input.keySet()) {
            if (input.get(key) > highestFrequency) {
                bestKey = key;
                highestFrequency = input.get(key);
            }
        }
        return bestKey;
    }
}