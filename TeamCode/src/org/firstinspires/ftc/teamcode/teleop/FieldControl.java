package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2D;

@TeleOp(name = "FieldControl", group = "MecanumBot")
public class FieldControl extends OpMode {
    DriveTrain dt;
    BetterGamepad gp1, gp2;
    Servo servo;
    boolean stoggle = false;

    Pose lastPose = new Pose(0, 0, 0);

    @Override
    public void init() {
        dt = new DriveTrain(hardwareMap);

        gp1 = new BetterGamepad(gamepad1);
        gp2 = new BetterGamepad(gamepad2);

        servo = hardwareMap.servo.get("back_servo");

        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < 1) {
            Vector2D p = (new Vector2D(0, 1)).rotated(-dt.getHeading());
            dt.setDrivePowers(p, 1);
            dt.updateOdometry();
        }

        /*timer.reset();
        dt.setDrivePowers(0, 0, 1);
        while (timer.seconds() < 1)
            dt.updateOdometry();*/

        dt.setDrivePowers(0, 0, 0);

        telemetry.addData("Max Linear Velocity", "%f/s", dt.getPosition().y);
        telemetry.addData("Max Angular Velocity", "%f/s", dt.getHeading());
        telemetry.update();

        /*try (var scope = HardwareTaskScope.open()) {
            var sa = new ServoAction(servo, 1, 1000);
            scope.fork(() -> {
                while (sa.run()) Thread.sleep(1);
            });
            scope.fork(() -> {
                while (dt.getPose().position.subtracted(new Vector2D(0, 36)).length() > 0.5) {
                    dt.setDrivePowers(0, 1, 0);
                    dt.updateOdometry();
                }
                dt.setDrivePowers(0, 0, 0);
            });
            scope.join();
        } catch (InterruptedException e) {
            System.out.println("scope interrupted :(");
        } finally {
            telemetry.addLine("async stuff done");
            telemetry.update();
        }*/
    }

    @Override
    public void loop() {
        dt.updateOdometry();
        gp1.update(gamepad1);

        dt.setDrivePowers(gp1.leftStickVector().rotated(-dt.getHeading()), gp1.rightStickVector().x);

        if (gp1.aPressed())
            stoggle = !stoggle;

        servo.setPosition(stoggle ? 0.5 : 0);

        Pose curPose = dt.getPose();
        Pose velocity = new Pose(curPose.position.x - lastPose.position.x, curPose.position.y - lastPose.position.y, curPose.heading - lastPose.heading);
        lastPose = curPose;
        telemetry.addData("x vel", velocity.position.x);
        telemetry.addData("y vel", velocity.position.y);
        telemetry.addData("h vel", velocity.heading);
        //telemetry.update();
    }
}