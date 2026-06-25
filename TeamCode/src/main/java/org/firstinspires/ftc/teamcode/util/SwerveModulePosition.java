package org.firstinspires.ftc.teamcode.util;

public class SwerveModulePosition {
    public double distanceMeters;
    public Rotation2d angle;

    public SwerveModulePosition(double distanceMeters, Rotation2d angle) {
        this.distanceMeters = distanceMeters;
        this.angle = angle;
    }

    public SwerveModulePosition() {
        this(0, new Rotation2d(0));
    }

    public SwerveModulePosition copy() {
        return new SwerveModulePosition(distanceMeters, angle);
    }

    public SwerveModulePosition interpolate(SwerveModulePosition endValue, double t) {
        if (t < 0) return this;
        if (t > 1) return endValue;
        double newDistance = distanceMeters + (endValue.distanceMeters - distanceMeters) * t;
        Rotation2d newAngle = angle.plus(new Rotation2d(endValue.angle.minus(angle).getRadians() * t));
        return new SwerveModulePosition(newDistance, newAngle);
    }

    @Override
    public String toString() {
        return String.format(java.util.Locale.US, "SwerveModulePosition(Distance: %.2f m, Angle: %s)", distanceMeters, angle);
    }
}
