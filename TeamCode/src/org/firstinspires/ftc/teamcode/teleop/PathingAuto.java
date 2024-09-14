package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Pathing.Actions.ServoAction;
import org.firstinspires.ftc.teamcode.subsystems.Pathing.Actions.WaitAction;
import org.firstinspires.ftc.teamcode.subsystems.Pathing.Path;
import org.firstinspires.ftc.teamcode.subsystems.Pathing.Waypoint;

@TeleOp(name = "Pathing", group = "Mecanum Bot")
public class PathingAuto extends OpMode {
    DriveTrain dt;
    Path path;

    @Override
    public void init() {
        dt = new DriveTrain(hardwareMap);//, new Pose(-62, 14, 0));

        Servo servo = hardwareMap.servo.get("back_servo");

        path = new Path(new Waypoint(dt.getPose()),
                        // -30, 48, 0
                        new Waypoint(34, 34, 0, new WaitAction(1500)),
                        new Waypoint(32, -4, Math.toRadians(180), new ServoAction(servo, 1, 1000)),
                        new Waypoint(53, 0, Math.toRadians(180)),
                        new Waypoint(53, -73, Math.toRadians(180), new WaitAction(2000)),
                        new Waypoint(53, 24, 0),
                        new Waypoint(34, 34, 0, new WaitAction(1500)),
                        new Waypoint(2, 34, 0),
                        new Waypoint(2, 46, 0));
        dt.startPath(path);
    }

    @Override
    public void loop() {
        dt.update();
        dt.debug(telemetry);
    }
}
