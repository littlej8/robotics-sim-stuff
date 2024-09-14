package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Localization.ParticleFilterLocalization.Particle;
import org.firstinspires.ftc.teamcode.util.Vector2D;

@TeleOp(name = "ParticleFilterTeleOp", group = "MecanumBot")
public class ParticleFilterTeleOp extends OpMode {
    DriveTrain dt;

    String[] sensorNames = {"front_distance", "right_distance", "back_distance", "left_distance"};
    DistanceSensor[] sensors = new DistanceSensor[4];

    @Override
    public void init() {
        dt = new DriveTrain(hardwareMap);

        for (int i = 0; i < 4; i++)
            sensors[i] = hardwareMap.get(DistanceSensor.class, sensorNames[i]);
    }

    @Override
    public void loop() {
        dt.updateOdometry();
        dt.setDrivePowers(new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y).rotated(-dt.getHeading()), gamepad1.right_stick_x);

        Particle p = new Particle(dt.getPose(), 1, 0);
        double[] outputs = p.getSensorOutputs();
        for (int i = 0; i < 4; i++) {
            telemetry.addData(sensorNames[i], sensors[i].getDistance(DistanceUnit.INCH));

            telemetry.addData("simulated-" + sensorNames[i], outputs[i]);
        }
    }
}
