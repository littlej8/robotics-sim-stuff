package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.util.FSFController;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2D;

@TeleOp(name = "TestFSFProfile", group = "MecanumBot")
public class TestFSFProfile extends OpMode {
    DriveTrain dt;
    Pose target = new Pose(48, 24, Math.toRadians(90));
    ElapsedTime timer;

    FSFController xFSF, yFSF, hFSF;
    MotionProfile xProfile, yProfile, hProfile;

    @Override
    public void init() {
        dt = new DriveTrain(hardwareMap);
        xFSF = new FSFController(1, 0.1);
        yFSF = new FSFController(1, 0.1);
        hFSF = new FSFController(10, 0.1);
        xProfile = new MotionProfile(0, target.position.x);
        yProfile = new MotionProfile(0, target.position.y);
        hProfile = new MotionProfile(0, target.heading, Math.toRadians(90), Math.toRadians(90));
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        dt.updateOdometry();

//        target.add((new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y)).multiplied(0.2));
//        xPID.setTarget(target.x);
//        yPID.setTarget(target.y);

        double[] xPosVel = xProfile.calculate(timer.seconds());
        double[] yPosVel = yProfile.calculate(timer.seconds());
        double[] hPosVel = hProfile.calculate(timer.seconds());
        xFSF.setTargets(xPosVel);
        yFSF.setTargets(yPosVel);
        hFSF.setTargets(hPosVel);

        Pose currentPose = dt.getPose();
        Pose currentVel = dt.getVelocity();
        double xPower = xFSF.update(currentPose.position.x, currentVel.position.x);
        double yPower = yFSF.update(currentPose.position.y, currentVel.position.y);
        double hPower = hFSF.update(currentPose.heading, currentVel.heading);

        Vector2D movePower = new Vector2D(xPower, yPower);
        movePower.rotate(-dt.getHeading());

        dt.setDrivePowers(movePower, hPower);

        telemetry.addData("Position", "(%.1f, %.1f)", currentPose.position.x, currentPose.position.y);
        telemetry.addData("Heading", "%.1f", Math.toDegrees(dt.getHeading()));
        telemetry.addData("TargetPose", "(%.1f, %.1f, %.1f)", xPosVel[0], yPosVel[0], Math.toDegrees(hPosVel[0]));
        telemetry.addData("Powers", "(%.1f, %.1f, %.1f)", movePower.x, movePower.y, hPower);
        telemetry.update();
    }
}