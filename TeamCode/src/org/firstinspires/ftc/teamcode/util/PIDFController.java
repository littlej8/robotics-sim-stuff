package org.firstinspires.ftc.teamcode.util;

public class PIDFController extends PIDController {
    double Kf;
    double Ka = 0;

    public PIDFController(double Kp, double Ki, double Kd, double Kf, double target) {
        super(Kp, Ki, Kd, target);
        this.Kf = Kf;
    }

    public PIDFController(double Kp, double Ki, double Kd, double Kf, double Ka, double target) {
        super(Kp, Ki, Kd, target);
        this.Kf = Kf;
        this.Ka = Ka;
    }

    @Override
    public double update(double current) {
        return super.update(current) + Kf * target;
    }

    public double update(double currentVel, double currentAccel) {
        return super.update(currentVel) + Kf * currentVel + Ka * currentAccel;
    }
}
