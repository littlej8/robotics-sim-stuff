package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2D;

@TeleOp(name = "PositionAnglePID", group = "MecanumBot")
public class  PositionAnglePID extends OpMode {
    DriveTrain dt;
    PIDController xPID;
    PIDController yPID;
    PIDController hPID;
    Pose target = new Pose(-24, 48, Math.toRadians(90));

    @Override
    public void init() {
        dt = new DriveTrain(hardwareMap);
        xPID = new PIDController(0.08, 0, 0, target.position.x);
        yPID = new PIDController(0.08, 0, 0, target.position.y);
        hPID = new PIDController(1, 0, 0, target.heading);
    }

    @Override
    public void loop() {
        dt.updateOdometry();

        //target.add((new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y)).multiplied(0.2));
        //xPID.setTarget(target.x);
        //yPID.setTarget(target.y);

        Vector2D currentPos = dt.getPosition();
        double currentHeading = dt.getHeading();

        double xPower = xPID.update(currentPos.x);
        double yPower = yPID.update(currentPos.y);
        double hPower = hPID.update(currentHeading);

        Vector2D movePower = new Vector2D(xPower, yPower);
        movePower.rotate(-dt.getHeading());

        dt.setDrivePowers(movePower, hPower);

        telemetry.addData("Position", "(%.1f, %.1f)", currentPos.x, currentPos.y);
        telemetry.addData("Heading", "%.1f", Math.toDegrees(dt.getHeading()));
        telemetry.addData("TargetPosition", "(%.1f, %.1f)", target.position.x, target.position.y);
        telemetry.addData("Powers", "(%.1f, %.1f, %.1f)", movePower.x, movePower.y, hPower);
        telemetry.update();
    }
}