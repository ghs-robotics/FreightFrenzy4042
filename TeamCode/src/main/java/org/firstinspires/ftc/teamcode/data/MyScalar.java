package org.firstinspires.ftc.teamcode.data;

import org.opencv.core.Scalar;

public class MyScalar extends Scalar {

    public MyScalar(double[] vals) {
        super(vals);
    }

    public MyScalar(double v0, double v1, double v2) {
        super(v0, v1, v2, 0);
    }

    public boolean inRange(MyScalar lower, MyScalar upper) {
        return (this.greaterThan(lower) && this.lessThan(upper));
    }

    public boolean lessThan(MyScalar other) {
        for (int i = 0; i < 3; i++) {
            if (other.val[i] <= this.val[i]) {
                return false;
            }
        }
        return true;
    }

    public boolean greaterThan(MyScalar other) {
        for (int i = 0; i < 3; i++) {
            if (other.val[i] >= this.val[i]) {
                return false;
            }
        }
        return true;
    }

    public double getHue() {
        return val[0];
    }

    public double getSaturation() {
        return val[1];
    }

    public double getValue() {
        return val[2];
    }
}
