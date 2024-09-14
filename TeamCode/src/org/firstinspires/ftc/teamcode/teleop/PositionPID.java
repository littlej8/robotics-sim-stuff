package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Vector2D;

@TeleOp(name = "PositionPID", group = "MecanumBot")
public class PositionPID extends OpMode {
    DriveTrain dt;
    PIDController xPID;
    PIDController yPID;
    Vector2D target = new Vector2D(144, 48);

    @Override
    public void init() {
        dt = new DriveTrain(hardwareMap);
        xPID = new PIDController(1, 0, 0, target.x);
        yPID = new PIDController(1, 0, 0, target.y);
    }

    @Override
    public void loop() {
        dt.updateOdometry();

        target.add((new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y)).multiplied(0.2));
        xPID.setTarget(target.x);
        yPID.setTarget(target.y);

        Vector2D currentPos = dt.getPosition();
        double xPower = xPID.update(currentPos.x);
        double yPower = yPID.update(currentPos.y);

        Vector2D movePower = new Vector2D(xPower, yPower);
        movePower.rotate(-dt.getHeading());

        dt.setDrivePowers(movePower, 0);

        telemetry.addData("Position", "(%.1f, %.1f)", currentPos.x, currentPos.y);
        telemetry.addData("Heading", "%.1f", Math.toDegrees(dt.getHeading()));
        telemetry.addData("TargetPosition", "(%.1f, %.1f)", target.x, target.y);
        telemetry.addData("Powers", "(%.1f, %.1f)", movePower.x, movePower.y);
        telemetry.update();
    }
}
