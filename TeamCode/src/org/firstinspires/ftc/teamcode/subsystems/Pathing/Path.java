package org.firstinspires.ftc.teamcode.subsystems.Pathing;

import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

// A Pure Pursuit Path
public class Path {
    List<Waypoint> points = new ArrayList<>();
    Pose finalPoint, lastIntersection;
    double followDistance = 1; // needs to be tuned
    int currentPointIndex = 1;

    public Path(Waypoint... points) throws IllegalArgumentException {
        if (points.length < 2)
            throw new IllegalArgumentException("Must pass 2 or more points to construct a path.");

        this.points.addAll(Arrays.asList(points));
        finalPoint = this.points.get(this.points.size() - 1);
    }

    public void setFollowDistance(double d) {
        followDistance = d;
    }

    public boolean finished(Pose current, double positionTolerance, double headingTolerance) {
        double dx = Math.abs(current.position.x - finalPoint.position.x);
        double dy = Math.abs(current.position.y - finalPoint.position.y);
        double dh = Math.abs(current.heading - finalPoint.heading);

        return (dx < positionTolerance) && (dy < positionTolerance) && (dh < headingTolerance);
    }

    public void start() {
        currentPointIndex = 1;
        lastIntersection = points.get(0);
    }

    private Pose correctedTargetPoint(Pose p) {
        Pose p1 = points.get(currentPointIndex);

        return new Pose(p1.position.x, p1.position.y, p1.heading);
    }

    public Pose update(Pose currentPosition) {
        Waypoint cur = points.get(currentPointIndex);
        if (cur.position.subtracted(currentPosition.position).length() < followDistance) {
            if (!cur.hasAction())       // no action at current waypoint
                currentPointIndex++;
            else if (!cur.runAction())  // action finished
                currentPointIndex++;
            else                        // keep target as current point until action is done
                return cur.getPose();
        }

        if (currentPointIndex > points.size() - 1) {
            currentPointIndex = points.size() - 1;
            return finalPoint;
        }

        Pose p1 = points.get(currentPointIndex - 1);
        Pose p2 = points.get(currentPointIndex);
        List<Pose> intersections = lineCircleIntersection(currentPosition, followDistance, p1, p2);

        if (intersections.isEmpty())
            return correctedTargetPoint(lastIntersection);
        else if (intersections.size() == 1) {
            lastIntersection = intersections.get(0);
            return correctedTargetPoint(intersections.get(0));
        } else {
            double d1 = p2.position.subtracted(intersections.get(0).position).length();
            double d2 = p2.position.subtracted(intersections.get(1).position).length();
            lastIntersection = (d1 < d2) ? intersections.get(0) : intersections.get(1);
            return correctedTargetPoint(lastIntersection);
        }
    }

    private static List<Pose> lineCircleIntersection(Pose circleCenter, double radius, Pose linePoint1, Pose linePoint2) {
        // This method was lifted from Team 11115 Gluten Free's code.

        double baX = linePoint2.position.x - linePoint1.position.x;
        double baY = linePoint2.position.y - linePoint1.position.y;
        double caX = circleCenter.position.x - linePoint1.position.x;
        double caY = circleCenter.position.y - linePoint1.position.y;

        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;

        double pBy2 = bBy2 / a;
        double q = c / a;

        double disc = pBy2 * pBy2 - q;
        if (disc < 0) {
            return Collections.emptyList();
        }

        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;

        List<Pose> allPoints = null;

        Pose p1 = new Pose(linePoint1.position.x - baX * abScalingFactor1, linePoint1.position.y - baY * abScalingFactor1);
        if (disc == 0) {
            allPoints = Collections.singletonList(p1);
        }

        if (allPoints == null) {
            Pose p2 = new Pose(linePoint1.position.x - baX * abScalingFactor2, linePoint1.position.y - baY * abScalingFactor2);
            allPoints = Arrays.asList(p1, p2);
        }

        double maxX = Math.max(linePoint1.position.x, linePoint2.position.x);
        double maxY = Math.max(linePoint1.position.y, linePoint2.position.y);
        double minX = Math.min(linePoint1.position.x, linePoint2.position.x);
        double minY = Math.min(linePoint1.position.y, linePoint2.position.y);

        List<Pose> boundedPoints = new ArrayList<Pose>();

        for (Pose point : allPoints) {

            if (point.position.x <= maxX && point.position.x >= minX)
                if (point.position.y <= maxY && point.position.y >= minY)
                    boundedPoints.add(point);

        }

        return boundedPoints;
    }
}
