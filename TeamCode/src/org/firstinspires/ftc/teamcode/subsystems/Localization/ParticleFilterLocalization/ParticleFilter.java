package org.firstinspires.ftc.teamcode.subsystems.Localization.ParticleFilterLocalization;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

public class ParticleFilter {
    ArrayList<Particle> particles = new ArrayList<>();
    private final Random random = new Random();
    int dimensions;

    public void add(Particle particle) {
        particles.add(particle);
    }

    public void clear() {
        particles.clear();
    }

    public ArrayList<Particle> getParticles() {
        return particles;
    }

    public void initParticles(int numParticles, Pose startingPose, double[] constraints) {
        for (int i = 0; i < numParticles; i++) {
            double[] deviances = new double[dimensions];

            for (int j = 0; j < dimensions; j++) {
                deviances[j] = ThreadLocalRandom.current().nextDouble(constraints[j*2], constraints[j*2+1]);
            }

            add(new Particle(startingPose.plus(new Pose(deviances[0], deviances[1], deviances[2])), 1, i));
        }
    }

    public void translateParticles(Pose translation) {
        int index = 0;

        for (Particle p : particles) {
            p.setPose(p.getPose().plus(translation));
            particles.set(index, p);
            index++;
        }
    }

    public void weighParticles(DistanceSensor[] sensors) {

    }

    public void resampleParticles(double[] resamplingDeviances) {
        int numParticles = particles.size();
        ArrayList<Particle> newParticles = new ArrayList<>(numParticles);

        double totalWeight = 0.0;

        for (Particle p : particles)
            totalWeight += p.getWeight();

        double stepSize = totalWeight / numParticles;
        double position = random.nextDouble() * stepSize;

        int index = 0;
        double cumulativeWeight = particles.get(0).getWeight();

        for (int i = 0; i < numParticles; i++) {
            while (position > cumulativeWeight && index < numParticles - 1) {
                index++;
                cumulativeWeight += particles.get(index).getWeight();
            }

            newParticles.add(new Particle(
                addGaussianNoise(particles.get(index).getPose(), resamplingDeviances),
                1.0,
                ThreadLocalRandom.current().nextInt()
            ));

            position += stepSize;
        }

        particles = newParticles;
    }

    public Particle getBestParticle() {
        double highestWeight = 0;
        Particle bestParticle = null;

        for (Particle p : particles) {
            double particleWeight = p.getWeight();
            if (particleWeight > highestWeight) {
                bestParticle = p;
                highestWeight = particleWeight;
            }
        }

        return bestParticle;
    }

    public Pose addGaussianNoise(Pose pose, double[] deviances) {
        return new Pose(
                random.nextGaussian() * deviances[0] + pose.position.x,
                random.nextGaussian() * deviances[1] + pose.position.y,
                random.nextGaussian() * deviances[2] + pose.heading
        );
    }
}
