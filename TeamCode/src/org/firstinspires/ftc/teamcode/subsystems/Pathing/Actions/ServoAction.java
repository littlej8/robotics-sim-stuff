package org.firstinspires.ftc.teamcode.subsystems.Pathing.Actions;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoAction implements Action {
    boolean started = false;
    ElapsedTime timer;
    long wait;
    Servo servo;
    double position;

    public ServoAction(Servo servo, double pos, long waitMillis) {
        this.servo = servo;
        position = pos;
        wait = waitMillis;
    }

    @Override
    public boolean run() {
        if (!started) {
            timer = new ElapsedTime();
            started = true;
        }

        servo.setPosition(position * (timer.milliseconds() / wait));

        return timer.milliseconds() < wait;
    }
}
