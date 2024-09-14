package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Localization.DeadReckoning.MecanumKinematics;
import org.firstinspires.ftc.teamcode.subsystems.Localization.DeadReckoning.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Pathing.Path;
import org.firstinspires.ftc.teamcode.subsystems.Pathing.PathFollower;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class DriveTrain {
    String[] motorNames = new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
    List<DcMotorEx> motors = new ArrayList<>();

    IMU imu;

    //Localizer localizer;
    Odometry odometry;
    PathFollower follower;

    public DriveTrain(HardwareMap hwMap) {
        this(hwMap, new Pose(0, 0, 0));
    }

    public DriveTrain(HardwareMap hwMap, Pose startPose) {
        for (String motor : motorNames)
            motors.add(hwMap.get(DcMotorEx.class, motor));

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motors.get(0).setDirection(DcMotor.Direction.REVERSE);
        motors.get(1).setDirection(DcMotor.Direction.REVERSE);

        /*
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
         */

        imu = hwMap.get(IMU.class, "imu");

        /*
        localizer = new Localizer(motors, imu);
        localizer.setPositionHeading(new Vector2D(0, 0), 0);
        */

        double[] wheelPositions = new double[4];
        wheelPositions[0] = motors.get(1).getCurrentPosition();
        wheelPositions[1] = motors.get(2).getCurrentPosition();
        wheelPositions[2] = motors.get(0).getCurrentPosition();
        wheelPositions[3] = motors.get(3).getCurrentPosition();

        MecanumKinematics kinematics = new MecanumKinematics(
            2,
            1120,
            new Vector2D(7, 8), //fl
            new Vector2D(7, -8), //fr
            new Vector2D(-7, 8), //bl
            new Vector2D(-7, -8)  //br
        );
        odometry = new Odometry(
                kinematics,
                imu,
                wheelPositions,
                new Pose(0, 0, 0)
        );
        odometry.setPose(startPose);
        //odometry.setUseIMU(false);

        follower = new PathFollower(this);
    }

    public void setDrivePowers(double x, double y, double a) {
        setDrivePowers(new Vector2D(x, y), a);
    }

    public void setDrivePowers(Vector2D t, double a) {
        double[] p = new double[4];
        p[0] = -t.x + t.y - a;
        p[1] = t.x + t.y - a;
        p[2] = -t.x + t.y + a;
        p[3] = t.x + t.y + a;

        double max = Math.max(1, Math.max(Math.abs(p[0]), Math.max(Math.abs(p[1]), Math.max(Math.abs(p[2]), Math.abs(p[3])))));
        if (max > 1) for (int i = 0; i < 4; i++) p[i] /= max;
        for (int i = 0; i < 4; i++) motors.get(i).setPower(p[i]);
    }

    public void updateOdometry() {
        //localizer.update();
        double[] wheelPositions = new double[4];
        wheelPositions[0] = motors.get(1).getCurrentPosition();
        wheelPositions[1] = motors.get(2).getCurrentPosition();
        wheelPositions[2] = motors.get(0).getCurrentPosition();
        wheelPositions[3] = motors.get(3).getCurrentPosition();

        odometry.update(wheelPositions);
    }

    public void startPath(Path p) {
        follower.startPath(p);
    }

    public void updateFollower() {
        follower.follow();
    }

    public void update() {
        updateOdometry();
        //if (follower.followingPath())
        updateFollower();
    }

    public void debug(Telemetry t) {
        follower.debug(t);
    }

    public Vector2D getPosition() {
        //return localizer.getPosition();
        Vector2D pos = odometry.getPose().position;
        return new Vector2D(-pos.y, pos.x);
    }

    public double getHeading() {
        // return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        //return localizer.getHeading();
        return odometry.getPose().heading;
    }

    public Pose getPose() {
        return new Pose(getPosition(), getHeading());
    }

    public Pose getVelocity() {
        return odometry.getVelocity();
    }
}
