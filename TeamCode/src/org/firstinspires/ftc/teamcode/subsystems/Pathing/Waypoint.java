package org.firstinspires.ftc.teamcode.subsystems.Pathing;

import org.firstinspires.ftc.teamcode.subsystems.Pathing.Actions.Action;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2D;

// don't ask why the subclass stores the parent class instance just accept it
public class Waypoint extends Pose {
    private Action action = null;
    private Pose pose;

    public Waypoint(Pose p, Action a) {
        super(p.position, p.heading);
        this.pose = p;
        this.action = a;
    }

    public Waypoint(Vector2D position, double heading, Action a) {
        this(new Pose(position, heading), a);
    }

    public Waypoint(double x, double y, double h, Action a) {
        this(new Pose(x, y, h), a);
    }

    public Waypoint(Pose p) {
        super(p.position, p.heading);
    }

    public Waypoint(Vector2D position, double heading) {
        super(position, heading);
    }

    public Waypoint(double x, double y, double h) {
        super(x, y, h);
    }

    public Waypoint(double x, double y) {
        super(x, y);
    }

    public Pose getPose() {
        return pose;
    }

    public boolean runAction() {
        return action.run();
    }

    public boolean hasAction() {
        return action != null;
    }
}
