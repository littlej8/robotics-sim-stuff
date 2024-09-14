package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Vector2D;

@TeleOp(name = "AnglePID", group = "MecanumBot")
public class AnglePID extends OpMode {
    DriveTrain dt;
    PIDController anglePID;
    double target = Math.toRadians(0);

    @Override
    public void init() {
        dt = new DriveTrain(hardwareMap);
        anglePID = new PIDController(5, 0, 0, target);
        anglePID.setWrapError(true);
    }

    @Override
    public void loop() {
        dt.updateOdometry();

        target -= Math.toRadians(gamepad1.right_stick_x) / 5;
        anglePID.setTarget(target);

        double currentHeading = dt.getHeading();
        double turnPower = anglePID.update(currentHeading);

        Vector2D movePower = new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        movePower.rotate(-currentHeading);

        dt.setDrivePowers(movePower, turnPower);

        telemetry.addData("Position", "(%.1f, %.1f)", dt.getPosition().x, dt.getPosition().y);
        telemetry.addData("Heading", " %.1f", Math.toDegrees(currentHeading));
        telemetry.addData("TargetHeading", " %.1f", Math.toDegrees(target));
        telemetry.addData("TurnPower", "%.1f", turnPower);
        telemetry.update();
    }
}
