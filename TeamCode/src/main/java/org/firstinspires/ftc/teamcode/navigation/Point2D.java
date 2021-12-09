package org.firstinspires.ftc.teamcode.navigation;

public class Point2D {
    public double x, y = 0.0;

    public Point2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // Vector add
    public Point2D add(Point2D other) {
        return new Point2D(this.x + other.x, this.y + other.y);
    }

    // Vector subtract
    public Point2D subtract(Point2D other) {
        return new Point2D(this.x - other.x, this.y - other.y);
    }

    public Point2D scale(double scalar) {
        return new Point2D(this.x * scalar, this.y * scalar);
    }

    public Point2D exponent(double exponent) {
        return new Point2D(Math.pow(this.x, exponent), Math.pow(this.y, exponent));
    }

    public double distanceTo(Point2D other) {
        Point2D sub = other.subtract(this);
        return Math.hypot(sub.x, sub.y);
    }

    public double length() {
        return Math.sqrt(x * x + y * y);
    }
}
