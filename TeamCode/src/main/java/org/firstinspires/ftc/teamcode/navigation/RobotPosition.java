package org.firstinspires.ftc.teamcode.navigation;

import android.graphics.Point;

public class RobotPosition {
    public Point2D position;
    public Double rotation;

    public RobotPosition(Point2D pos, Double rot) { //rot is in degrees
        this.position = pos;
        this.rotation = rot;
    }

    public RobotPosition(double x, double y, Double rot) {
        this(new Point2D(x, y), rot);
    }
}
