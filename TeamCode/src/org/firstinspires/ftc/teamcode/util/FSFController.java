package org.firstinspires.ftc.teamcode.util;

/* A Full State Feedback Controller */
public class FSFController {
    double[] targets, gains;

    public FSFController(double... gains) {
        this.gains = gains;
    }

    public FSFController(double[] gains, double[] targets) {
        if (gains.length != targets.length)
            throw new IllegalArgumentException("Gain and target vectors must be the same length.");

        this.gains = gains;
        this.targets = targets;
    }

    public void setGains(double... gains) {
        this.gains = gains;
    }

    public void setTargets(double... targets) {
        this.targets = targets;
    }

    public double update(double... currents) {
        if (currents.length != gains.length || currents.length != targets.length)
            throw new IllegalArgumentException("The current state vector must be the same length as state and gain vectors.");

        double u = 0;
        for (int i = 0; i < currents.length; i++)
            u += gains[i] * (targets[i] - currents[i]);
        return u;
    }
}
