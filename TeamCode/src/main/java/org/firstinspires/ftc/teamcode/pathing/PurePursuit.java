package org.firstinspires.ftc.teamcode.pathing;

import java.util.*;

public class PurePursuit {
    private float[] pos;
    private List<float[]> path;
    private float lookaheadDistance = 45;
    private float lookaheadDistanceDelta = 2.5f;
    private float pointRadius = 20;
    public void reset() {
        pos = new float[]{0, 0};
        path = new ArrayList<>();
    }
    public void update() {
        //get closest point in path
        float[] closest = pos;
        float closestDist = (float)Double.POSITIVE_INFINITY;
        for (int i = 0; i < path.size(); i++) {
            float currentDist = distanceTo(pos, path.get(i));
            if (currentDist > closestDist) {
                closest = path.get(i);
                closestDist = currentDist;
            }
        }
    }
    private float distanceTo(float[] a, float[] b) {
        return (float)Math.sqrt(Math.pow((a[0] - b[0]), 2) + Math.pow((a[1] - b[1]), 2));
    }
}
