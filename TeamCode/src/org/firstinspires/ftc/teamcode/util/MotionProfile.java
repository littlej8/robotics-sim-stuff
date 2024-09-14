package org.firstinspires.ftc.teamcode.util;

public class MotionProfile {
    double max_accel, max_vel;
    double start, end;

    public MotionProfile(double start, double end) {
        this(start, end, 12, 48);
    }

    public MotionProfile(double start, double end, double max_accel) {
        this(start, end, max_accel, 48);
    }

    // naming conventions are irrelevant;
    public MotionProfile(double start_p, double end_p, double max_accel_p, double max_vel_p) {
        start = start_p;
        end = end_p;
        max_accel = max_accel_p;
        max_vel = max_vel_p;
    }

    // returns a desired position and velocity for a specific time in the profile
    // velocity = starting velocity + accelerating * time
    // position = starting position + starting velocity * time + (1/2) * acceleration * time
    public double[] calculate(double elapsedTime) {
        if (start == end) // useless profile???
            return new double[]{start, 0};

        double distance = end - start;
        double half_distance = distance / 2;

        double accel_dt = max_vel / max_accel; // length of time to accelerate

        if ((0.5 * max_accel * accel_dt * accel_dt) > half_distance)
            accel_dt = Math.sqrt(half_distance / (0.5 * max_accel)); // accel time is shortened if we can't reach max velocity

        double accel_distance = 0.5 * max_accel * accel_dt * accel_dt; // distance traveled during acceleration

        max_vel = max_accel * accel_dt; // max velocity is lowered if we can't reach it in time

        double decel_dt = accel_dt; // length of time to decelerate is same as time to accelerate

        double cruise_distance = distance - 2 * accel_distance; // distance traveled during cruising
        double cruise_dt = cruise_distance / max_vel; // length of time to cruise at max velocity
        double decel_time = accel_dt + cruise_dt; // time to start decelerating

        double entire_dt = accel_dt + cruise_dt + decel_dt; // total length of time profile will take to complete

        if (elapsedTime > entire_dt) // profile is done
            return new double[]{distance, 0};
        else if (elapsedTime < accel_dt) // accelerating
            return new double[]{0.5 * max_accel * elapsedTime * elapsedTime, max_accel * elapsedTime};
        else if (elapsedTime < decel_time) { // cruising
            double cruise_elapsed = elapsedTime - accel_dt; // time since started cruising
            return new double[]{accel_distance + max_vel * cruise_elapsed, max_vel};
        } else { // decelerating
            double decel_elapsed = elapsedTime - decel_time; // time since started decelerating
            return new double[]{accel_distance + cruise_distance + max_vel * decel_elapsed - 0.5 * max_accel * decel_elapsed * decel_elapsed, max_vel - max_accel * decel_elapsed};
        }
    }

    public void setMaxAcceleration(double max_accel) {
        this.max_accel = max_accel;
    }

    public void setMaxVelocity(double max_vel) {
        this.max_vel = max_vel;
    }
}