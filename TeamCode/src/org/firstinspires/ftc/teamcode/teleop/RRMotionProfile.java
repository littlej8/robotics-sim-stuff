package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2D;

@TeleOp(name="RRMotionProfile", group="Mecanum Bot")
public class RRMotionProfile extends OpMode {
    DriveTrain dt;
    MotionProfile p;
    PIDFController c;
    ElapsedTime timer;

    @Override
    public void init() {
        dt = new DriveTrain(hardwareMap);
        p = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0), new MotionState(24, 0), 24, 48);
        c = new PIDFController(1, 0, 0, 1, 1, 24);
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        dt.updateOdometry();

        Pose currentPose = dt.getPose();
        Pose vel = dt.getVelocity();
        MotionState yState = p.get(timer.seconds());
        c.setTargetIfDiff(yState.getV());
        Vector2D movePower = new Vector2D(0, c.update(vel.position.y));

        dt.setDrivePowers(movePower, 0);

        telemetry.addData("Position", "(%.1f, %.1f)", currentPose.position.x, currentPose.position.y);
        telemetry.addData("Heading", "%.1f", Math.toDegrees(dt.getHeading()));
        //telemetry.addData("TargetPose", "(%.1f, %.1f, %.1f)", xPosVel[0], yPosVel[0], Math.toDegrees(hPosVel[0]));
        telemetry.addData("Powers", "(%.1f, %.1f, %.1f)", movePower.x, movePower.y, 0.0);
        telemetry.update();
    }
}
