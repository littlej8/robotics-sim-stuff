package org.firstinspires.ftc.teamcode.subsystems.Pathing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2D;

public class PathFollower {
    DriveTrain dt;
    PIDController xPID, yPID, hPID;
    Path path;
    double translationalTolerance = 1, headingTolerance = 0.25;

    Pose target;
    Vector2D tpower = new Vector2D(0, 0);
    double hpower = 0;

    public static final double MAX_LINEAR_VELOCITY = 12.258076; // in/sec
    public static final double MAX_ANGULAR_VELOCITY = 0.809199; // rad/sec

    public PathFollower(DriveTrain dt) {
        this.dt = dt;
        xPID = new PIDController(.5, 0, 0, 0);
        yPID = new PIDController(.5, 0, 0, 0);
        hPID = new PIDController(5, 0, 0, 0);
        hPID.setWrapError(true);
    }

    public void setTranslationalTolerance(double t) {
        translationalTolerance = t;
    }

    public void setHeadingTolerance(double t) {
        headingTolerance = t;
    }

    public void startPath(Path p) {
        path = p;
        path.start();
    }

    public boolean followingPath() {
        if (path != null)
            return !path.finished(dt.getPose(), translationalTolerance, headingTolerance);
        return false;
    }

    public void debug(Telemetry t) {
        Pose currentPos = dt.getPose();
        double timeToMove = (target.position.distTo(currentPos.position)) / MAX_LINEAR_VELOCITY;
        double timeToTurn = Math.abs(AngleUtils.normalizeRadians(target.heading - currentPos.heading)) / MAX_ANGULAR_VELOCITY;

        t.addData("Current Position", "(%.2f, %.2f)", dt.getPosition().x, dt.getPosition().y);
        t.addData("Current Heading", "%.1f", Math.toDegrees(dt.getHeading()));
        t.addData("Target Position", "(%.2f, %.2f)", target.position.x, target.position.y);
        t.addData("Target Heading", "%.1f", Math.toDegrees(target.heading));
        t.addData("Drive Powers", "(%.3f, %.3f, %.3f)", tpower.x, tpower.y, hpower);
        t.addData("Time to Move/Turn", "(%.3f, %.3f)", timeToMove, timeToTurn);
        t.update();
    }

    public boolean finished() {
        return path.finished(dt.getPose(), translationalTolerance, headingTolerance);
    }

    public void follow() {
        Pose currentPos = dt.getPose();

        target = path.update(currentPos);

        xPID.setTargetIfDiff(target.position.x);
        yPID.setTargetIfDiff(target.position.y);
        hPID.setTargetIfDiff(target.heading);

        Vector2D translationalPowers = new Vector2D(xPID.update(currentPos.position.x), yPID.update(currentPos.position.y));

        translationalPowers.rotate(-currentPos.heading);

        double rotationalPower = hPID.update(currentPos.heading);

        double timeToMove = (target.position.distTo(currentPos.position)) / MAX_LINEAR_VELOCITY;
        double timeToTurn = Math.abs(AngleUtils.normalizeRadians(target.heading - currentPos.heading)) / MAX_ANGULAR_VELOCITY;

        if (timeToTurn > 0 && timeToMove > 0) {
            double ratio = timeToTurn / timeToMove;

            Vector2D thing = new Vector2D(target.position.distTo(currentPos.position), AngleUtils.normalizeRadians(target.heading - currentPos.heading));
            thing.normalize();
            thing.multiply(ratio);
        }

        tpower = translationalPowers;
        hpower = rotationalPower;

        dt.setDrivePowers(translationalPowers, rotationalPower);
    }
}
