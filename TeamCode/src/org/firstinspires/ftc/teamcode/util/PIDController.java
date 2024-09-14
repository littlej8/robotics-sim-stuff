package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    double Kp, Ki, Kd;
    double lastError, integralSum;
    double target;
    ElapsedTime timer;

    boolean wrapError = false;

    public PIDController(double Kp, double Ki, double Kd, double target) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.target = target;

        this.integralSum = 0;
        this.lastError = 0;

        timer = new ElapsedTime();
    }

    public void setTarget(double target) {
        this.target = target;
        this.integralSum = 0;
        this.lastError = 0;
    }

    public void setTargetIfDiff(double target) {
        if (this.target == target) return;
        this.target = target;
        this.integralSum = 0;
        this.lastError = 0;
    }

    public void setWrapError(boolean wrapError) {
        this.wrapError = wrapError;
    }

    public double update(double current) {
        double error = (wrapError) ? AngleUtils.normalizeRadians(target - current) : target - current;
        integralSum += error * timer.seconds();
        double d = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return error * Kp + integralSum * Ki + d * Kd;
    }
}
