package org.firstinspires.ftc.teamcode.subsystems.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2D;

import java.util.ArrayList;
import java.util.List;

/*
 * A differential drive path follower that assumes
 * constant forward momentum and only controls steering
 */
public class PurePursuitPathFollower {
    DriveTrain dt;
    double lookAhead;
    double linearGain = 0.5, turnGain = 1;
    List<Waypoint> path = new ArrayList<Waypoint>();

    int currentIndex;
    Vector2D lastIntersection;

    public PurePursuitPathFollower(DriveTrain dt, double lookAhead, Waypoint... path) {
        this.dt = dt;
        setLookAhead(lookAhead);
        setPath(path);
    }

    public PurePursuitPathFollower(DriveTrain dt, Waypoint... path) {
        this(dt, 1, path);
    }

    public PurePursuitPathFollower(DriveTrain dt) {
        this.dt = dt;
    }

    public void setPath(Waypoint... path) {
        this.path = List.of(path);
        currentIndex = 0;
        lastIntersection = this.path.get(0).position;
    }

    public void setLookAhead(double lookAhead) {
        this.lookAhead = lookAhead;
    }

    public void setGains(double linearGain, double turnGain) {
        this.linearGain = linearGain;
        this.turnGain = turnGain;
    }

    // runs until done
    public void followBlocking(LinearOpMode op) {
        while (!op.isStopRequested()) {
            followAsync();
        }
    }

    // sets required dt powers then returns for use in a state machine or similar async structure
    public void followAsync() {
        if (path.size() < 2)
            throw new IllegalStateException("Must have path with at least 2 points.");

        Pose curPose = dt.getPose();

        // find intersection that is farthest along the path
        Vector2D goal = lastIntersection;
        for (int i = currentIndex; i < path.size() - 1; i++) {
            Vector2D p1 = path.get(i).position;
            Vector2D p2 = path.get(i + 1).position;

            List<Vector2D> intersections = lineCircleIntersection(curPose.position, p1, p2);
            if (intersections.size() == 2) {
                double d1 = intersections.get(0).distTo(p2);
                double d2 = intersections.get(1).distTo(p2);
                goal = (d1 < d2) ? intersections.get(0) : intersections.get(1);
            } else if (intersections.size() == 1) {
                goal = intersections.get(0);
            }

            if (goal.distTo(p2) < curPose.position.distTo(p2)) {
                currentIndex = i;
                break;
            } else {
                currentIndex++;
            }
        }

        lastIntersection = goal;

        double linearError = Math.sqrt(
                Math.pow(goal.x - curPose.position.x, 2) +
                Math.pow(goal.y - curPose.position.y, 2)
        );
        double turnError = AngleUtils.normalizeRadians(Math.atan2(
                goal.y - curPose.position.y,
                goal.x - curPose.position.x
        ));

        dt.setDrivePowers(0, linearError, turnGain * turnError);
        dt.updateOdometry();
    }

    private List<Vector2D> lineCircleIntersection(Vector2D curPos, Vector2D p1, Vector2D p2) {
        double curX = curPos.x;
        double curY = curPos.y;

        // offset points so circle is at (0, 0) for easier math
        double x1 = p1.x - curX;
        double y1 = p1.y - curY;
        double x2 = p2.x - curX;
        double y2 = p2.y - curY;

        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr = Math.sqrt(dx * dx + dy * dy);
        double D = x1 * y2 - x2 * y1;
        double discriminant = (lookAhead * lookAhead) * (dr * dr) - (D * D);

        // if negative then no solution
        if (discriminant >= 0) {
            Vector2D solution1 = new Vector2D(
                    (D * dy + sgn(dy) * dx * Math.sqrt(discriminant)) / (dr * dr),
                    (-D * dx + Math.abs(dy) * Math.sqrt(discriminant)) / (dr * dr)
            );
            Vector2D solution2 = new Vector2D(
                    (D * dy - sgn(dy) * dx * Math.sqrt(discriminant)) / (dr * dr),
                    (-D * dx - Math.abs(dy) * Math.sqrt(discriminant)) / (dr * dr)
            );

            double minX = Math.min(x1, x2);
            double maxX = Math.max(x1, x2);
            double minY = Math.min(y1, y2);
            double maxY = Math.max(y1, y2);

            // return the points between the two provided points
            List<Vector2D> solutions = new ArrayList<>();
            if (minX <= solution1.x && solution1.x <= maxX && minY <= solution1.y && solution1.y < maxY)
                solutions.add(solution1);
            if (minX <= solution2.x && solution2.x <= maxX && minY <= solution2.y && solution2.y < maxY)
                solutions.add(solution2);
            return solutions;
        } else
            return new ArrayList<>();
    }

    private double sgn(double n) {
        return (n >= 0) ? 1 : -1;
    }
}
