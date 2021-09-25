package org.firstinspires.ftc.teamcode.robot_components.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DuckSpinner {
    DcMotor motor = null;
    double muG;
    double radius;
    private ElapsedTime runtime = null;

    final double G_CONST = -9.8; // m/s^2
    final double DUCK_MASS = 0.01700971; // kg
    final double SPINNER_MOMENT_OF_INERTIA = 1000; // probably not accurate, needs tweaking
    final double MAX_MOTOR_TORQUE = 18.7;

    public DuckSpinner(DcMotor motor, double frictionCoefficient, double radius) {
        this.motor = motor;
        this.muG = frictionCoefficient * G_CONST;
        this.radius = radius;
    }

    public double calcAccel(double t) {
        double k = this.muG / this.radius;
        double rootK = Math.sqrt(k);
        double partTwo = t / (Math.pow(Math.cosh(rootK * t), 2));
        double alpha = 0.5 * ( (Math.tanh(rootK * t) / rootK) + partTwo);
        return alpha;
    }

    public double calcPower(double alpha) {
        double torque =  alpha / SPINNER_MOMENT_OF_INERTIA;
        return torque / MAX_MOTOR_TORQUE;
    }
    public void startTimer() {
        runtime.reset();
    }

}
