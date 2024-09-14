package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Pathing.PurePursuitPathFollower;
import org.firstinspires.ftc.teamcode.subsystems.Pathing.Waypoint;

@TeleOp(name = "TestPPPathing", group = "MecanumBot")
public class TestPPPathing extends OpMode {
    DriveTrain dt;
    PurePursuitPathFollower follower;

    @Override
    public void init() {
        dt = new DriveTrain(hardwareMap);
        follower = new PurePursuitPathFollower(dt);
        follower.setPath(
                new Waypoint(0, 8, 0),
                new Waypoint(2, 14, 0),
                new Waypoint(6, 18, 0),
                new Waypoint(12, 20, 0),
                new Waypoint(20, 20, 0)
        );
    }

    @Override
    public void loop() {
        follower.followAsync();
    }
}
