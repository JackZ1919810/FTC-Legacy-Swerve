package org.firstinspires.ftc.teamcode.util;

public class Rotation2d {
    public static final Rotation2d kZero = new Rotation2d(0);
    private final double radians;

    public Rotation2d(double radians) {
        this.radians = normalize(radians);
    }

    public static Rotation2d fromRadians(double radians) {
        return new Rotation2d(radians);
    }

    public static Rotation2d fromDegrees(double degrees) {
        return new Rotation2d(Math.toRadians(degrees));
    }

    public double getRadians() {
        return radians;
    }

    public double getDegrees() {
        return Math.toDegrees(radians);
    }

    public double cos() {
        return Math.cos(radians);
    }

    public double sin() {
        return Math.sin(radians);
    }

    public Rotation2d plus(Rotation2d other) {
        return new Rotation2d(radians + other.radians);
    }

    public Rotation2d minus(Rotation2d other) {
        return new Rotation2d(radians - other.radians);
    }

    public Rotation2d unaryMinus() {
        return new Rotation2d(-radians);
    }

    private static double normalize(double radians) {
        double normalized = radians % (2 * Math.PI);
        if (normalized > Math.PI) {
            normalized -= 2 * Math.PI;
        } else if (normalized <= -Math.PI) {
            normalized += 2 * Math.PI;
        }
        return normalized;
    }

    @Override
    public String toString() {
        return String.format(java.util.Locale.US, "Rotation2d(%.2f°)", getDegrees());
    }
}
