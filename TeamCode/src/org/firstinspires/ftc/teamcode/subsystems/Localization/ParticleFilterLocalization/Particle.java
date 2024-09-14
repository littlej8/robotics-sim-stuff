package org.firstinspires.ftc.teamcode.subsystems.Localization.ParticleFilterLocalization;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Pose;
import virtual_robot.config.Config;
import virtual_robot.util.AngleUtils;

public class Particle {
    Pose pose;
    double weight;
    int index;

    SimulatedDistanceSensor[] sensors = new SimulatedDistanceSensor[4];

    public class SimulatedDistanceSensor {

        private final double readingWhenOutOfRangeMM = 8200;
        private double distanceMM = readingWhenOutOfRangeMM;
        private static final double MIN_DISTANCE = 50; //mm
        private static final double MAX_DISTANCE = 1000; //mm
        private static final double MAX_OFFSET = 7.0 * Math.PI / 180.0;
        private static final double fieldWidth = 648;
        private static final double halfFieldWidth = fieldWidth / 2;

        private final double X_MIN, X_MAX, Y_MIN, Y_MAX;    //Need these to constrain field

        public SimulatedDistanceSensor(){
            X_MIN = 2.0 * (Config.X_MIN_FRACTION - 0.5) * halfFieldWidth;
            X_MAX = 2.0 * (Config.X_MAX_FRACTION - 0.5) * halfFieldWidth;
            Y_MIN = 2.0 * (Config.Y_MIN_FRACTION - 0.5) * halfFieldWidth;
            Y_MAX = 2.0 * (Config.Y_MAX_FRACTION - 0.5) * halfFieldWidth;
        }

        public synchronized double getDistance(DistanceUnit distanceUnit){
            double result;
            if (distanceMM < MIN_DISTANCE) result = MIN_DISTANCE - 1.0;
            else if (distanceMM > MAX_DISTANCE) result = readingWhenOutOfRangeMM;
            else result = distanceMM;
            return distanceUnit.fromMm(distanceMM);
        }

        public synchronized void updateDistance(double x, double y, double headingRadians){
            final double mmPerPixel = 144.0 * 25.4 / fieldWidth;

            final double piOver2 = Math.PI / 2.0;
            double temp = headingRadians / piOver2;
            int side = (int)Math.round(temp); //-2, -1 ,0, 1, or 2 (2 and -2 both refer to the bottom)
            double offset = Math.abs(headingRadians - (side * Math.PI / 2.0));
            if (offset > MAX_OFFSET) distanceMM = readingWhenOutOfRangeMM;
            else switch (side){
                case 2:
                case -2:
                    distanceMM = ((y - Y_MIN) * mmPerPixel) - 14;                  //BOTTOM
                    break;
                case -1:
                    distanceMM = (X_MAX - x) * mmPerPixel;         //RIGHT
                    break;
                case 0:
                    distanceMM = (Y_MAX - y) * mmPerPixel;         //TOP
                    break;
                case 1:
                    distanceMM = (x - X_MIN) * mmPerPixel;         //LEFT
                    break;
            }
        }
    }

    public Particle() {}

    public Particle(Pose pose, double weight, int index) {
        this.pose = pose;
        this.weight = weight;
        this.index = index;

        for (int i = 0; i < 4; i++) {
            sensors[i] = new SimulatedDistanceSensor();
        }
    }

    public double[] getSensorOutputs() {
        updateSensors();

        double[] outputs = new double[4];
        for (int i = 0; i < 4; i++) {
            outputs[i] = sensors[i].getDistance(DistanceUnit.INCH);
        }

        return outputs;
    }

    private void updateSensors() {
        final double piOver2 = Math.PI / 2.0;

        final double mmPerPixel = 144.0 * 25.4 / 648;
        double x = DistanceUnit.MM.fromInches(pose.position.x) / mmPerPixel;
        double y = DistanceUnit.MM.fromInches(pose.position.y) / mmPerPixel;
        x = 324 - x;
        y = y + 31.5;

        System.out.printf("%f, %f\n", x, y);

        for (int i = 0; i < 4; i++) {
            double sensorHeading = AngleUtils.normalizeRadians(pose.heading + i * piOver2);
            sensors[i].updateDistance(x - 9 * Math.sin(sensorHeading),
                    y + 9 * Math.cos(sensorHeading), sensorHeading);
        }
    }

    public void setPose(Pose newPose) {
        pose = newPose;
    }

    public void setWeight(double newWeight) {
        weight = newWeight;
    }

    public Pose getPose() {
        return pose;
    }

    public double getWeight() {
        return weight;
    }

    public int getIndex() {
        return index;
    }
}
