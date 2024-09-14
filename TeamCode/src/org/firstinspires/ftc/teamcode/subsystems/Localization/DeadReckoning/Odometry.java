package org.firstinspires.ftc.teamcode.subsystems.Localization.DeadReckoning;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Rotation2d;

public class Odometry {
    private final MecanumKinematics kinematics;
    private Pose poseInches, velocityInches;

    private IMU gyro;
    private Rotation2d gyroOffset;
    private Rotation2d previousAngle;
    private final double[] previousWheelPositions;

    private boolean useIMU = true;

    public Odometry(
            MecanumKinematics kinematics,
            IMU imu,
            double[] wheelPositions,
            Pose initialPoseInches) {
        this.kinematics = kinematics;
        this.poseInches = initialPoseInches;
        this.gyro = imu;
        double gyroAngle = gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        this.gyroOffset = new Rotation2d(initialPoseInches.heading - gyroAngle);
        this.previousAngle = new Rotation2d(gyroAngle);
        this.previousWheelPositions = wheelPositions.clone();
    }

    public void setUseIMU(boolean useIMU) {
        this.useIMU = useIMU;
    }

    public void setPose(Pose pose) {
        poseInches = pose;
    }

    public Pose getPose() {
        return poseInches;
    }

    public Pose getVelocity() {
        return velocityInches;
    }

    public Pose update(double[] wheelPositions) {
        Rotation2d gyroAngle = new Rotation2d(gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        Rotation2d angle = (useIMU) ? gyroAngle.plus(gyroOffset) : new Rotation2d(poseInches.heading);

        Pose twist = kinematics.toTwist(previousWheelPositions, wheelPositions);
        twist.heading = angle.minus(previousAngle).getRadians();
        velocityInches = new Pose(twist.position.x, twist.position.y, twist.heading);

        Pose newPose = poseInches.exp(twist);

        for (int i = 0; i < 4; i++)
            previousWheelPositions[i] = wheelPositions[i];

        previousAngle = angle;
        poseInches = new Pose(newPose.position, angle.getRadians());

        return poseInches;
    }
}
