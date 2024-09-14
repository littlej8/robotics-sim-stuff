package org.firstinspires.ftc.teamcode.subsystems.Pathing.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitAction implements Action {
    boolean started = false;
    ElapsedTime timer;
    long wait;

    public WaitAction(long waitMillis) {
        wait = waitMillis;
    }

    @Override
    public boolean run() {
        if (!started) {
            timer = new ElapsedTime();
            started = true;
        }

        return timer.milliseconds() < wait;
    }
}
