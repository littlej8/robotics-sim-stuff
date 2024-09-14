package org.firstinspires.ftc.teamcode.subsystems.Localization;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class Localizer {
    List<DcMotorEx> encoders = new ArrayList<>(); // bl, fl, fr, br
    IMU imu;

    int[] encoderPos = new int[4], lastEncoderPos = new int[4];
    Vector2D position, lastPosition;
    double heading, lastHeading;
    ElapsedTime dt;

    // all units in inches!!!
    public static final double BOT_WIDTH = 18;
    public static final double WHEEL_RADIUS = 2;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;
    public static final double WHEEL_TICKS_PER_REV = 1120;
    public static final double WHEEL_IN_PER_TICK = WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_REV;
    public static final double WHEEL_X_DIST = 8;
    public static final double WHEEL_Y_DIST = 7;

    public Localizer(List<DcMotorEx> encoders, IMU imu) {
        this.encoders = encoders;
        this.imu = imu;

        position = new Vector2D(0, 0);
        heading = 0;

        lastPosition = new Vector2D(0, 0);
        lastHeading = 0;
    }

    public void setPositionHeading(Vector2D position, double heading) {
        this.position = position;
        this.heading = heading;
    }

    public void update() {
        lastEncoderPos = encoderPos.clone();
        lastPosition = new Vector2D(position.x, position.y);
        lastHeading = heading;

        int[] d = new int[4];
        for (int i = 0; i < 4; i++) {
            encoderPos[i] = encoders.get(i).getCurrentPosition();
            d[i] = encoderPos[i] - lastEncoderPos[i];
        }

        double currentHeading = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        // bl, fl, fr, br
        int fl = d[1];
        int fr = d[2];
        int bl = d[0];
        int br = d[3];

        double robot_dy = (fl + fr + bl + br) * (WHEEL_RADIUS / 4) * WHEEL_IN_PER_TICK;
        double robot_dx = -(-fl + fr + bl - br) * (WHEEL_RADIUS / 4) * WHEEL_IN_PER_TICK;

        Vector2D field_dpos = (new Vector2D(robot_dx, robot_dy)).rotated(currentHeading).multiplied(0.5);

        //double dh = (-fl + fr - bl + br) * (WHEEL_RADIUS / (4 * (WHEEL_X_DIST + WHEEL_Y_DIST)));
        double dh = currentHeading - lastHeading;

        Pose twist = new Pose(field_dpos, dh);
        Pose newPose = (new Pose(position, heading)).exp(twist);

        position = position.added(field_dpos);//newPoint.position;
        heading = currentHeading;
    }

    public Vector2D getPosition() {
        return new Vector2D(position.x, position.y);
    }

    public double getHeading() {
        return heading;
    }

    public Vector2D getVelocity() {
        return position.subtracted(lastPosition);
    }

    public double getAngularVelocity() {
        return heading - lastHeading;
    }
}
