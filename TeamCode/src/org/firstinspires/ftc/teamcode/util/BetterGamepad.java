package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class BetterGamepad {
    Gamepad cur = new Gamepad(), prev = new Gamepad();

    public BetterGamepad(Gamepad gp) {
        cur.copy(gp);
        prev.copy(gp);
    }

    public void update(Gamepad gp) {
        prev.copy(cur);
        cur.copy(gp);
    }

    public Vector2D leftStickVector() {
        return new Vector2D(cur.left_stick_x, cur.left_stick_y);
    }

    public Vector2D rightStickVector() {
        return new Vector2D(-cur.right_stick_x, cur.right_stick_y);
    }

    public boolean xPressed() {
        return !prev.x && cur.x;
    }

    public boolean yPressed() {
        return !prev.y && cur.y;
    }

    public boolean aPressed() {
        return !prev.a && cur.a;
    }

    public boolean bPressed() {
        return !prev.b && cur.b;
    }

    public boolean circlePressed() {
        return !prev.circle && cur.circle;
    }

    public boolean crossPressed() {
        return !prev.cross && cur.cross;
    }

    public boolean trianglePressed() {
        return !prev.triangle && cur.triangle;
    }

    public boolean squarePressed() {
        return !prev.square && cur.square;
    }

    public boolean sharePressed() {
        return !prev.share && cur.share;
    }

    public boolean optionsPressed() {
        return !prev.options && cur.options;
    }

    public boolean psPressed() {
        return !prev.ps && cur.ps;
    }

    public boolean dpadUpPressed() {
        return !prev.dpad_up && cur.dpad_up;
    }

    public boolean dpadDownPressed() {
        return !prev.dpad_down && cur.dpad_down;
    }

    public boolean dpadLeftPressed() {
        return !prev.dpad_left && cur.dpad_left;
    }

    public boolean dpadRightPressed() {
        return !prev.dpad_right && cur.dpad_right;
    }

    public boolean backPressed() {
        return !prev.back && cur.back;
    }

    public boolean guidePressed() {
        return !prev.guide && cur.guide;
    }

    public boolean startPressed() {
        return !prev.start && cur.start;
    }

    public boolean leftBumperPressed() {
        return !prev.left_bumper && cur.left_bumper;
    }

    public boolean rightBumperPressed() {
        return !prev.right_bumper && cur.right_bumper;
    }

    public boolean leftStickButtonPressed() {
        return !prev.left_stick_button && cur.left_stick_button;
    }

    public boolean rightStickButtonPressed() {
        return !prev.right_stick_button && cur.right_stick_button;
    }

    public boolean leftTriggerPressed() {
        return (prev.left_trigger < 0.5) && (cur.left_trigger > 0.5);
    }

    public boolean rightTriggerPressed() {
        return (prev.right_trigger < 0.5) && (cur.right_trigger > 0.5);
    }
}
