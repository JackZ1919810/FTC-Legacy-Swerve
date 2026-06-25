package org.firstinspires.ftc.teamcode.util;

public class Twist2d {
    public double dx;
    public double dy;
    public double dtheta;

    public Twist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    public Twist2d() {
        this(0, 0, 0);
    }

    @Override
    public String toString() {
        return String.format(java.util.Locale.US, "Twist2d(dx: %.2f, dy: %.2f, dtheta: %.2f)", dx, dy, dtheta);
    }
}
