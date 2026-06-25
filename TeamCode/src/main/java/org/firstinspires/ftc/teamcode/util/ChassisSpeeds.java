package org.firstinspires.ftc.teamcode.util;

public class ChassisSpeeds {
    public double vxMetersPerSecond;
    public double vyMetersPerSecond;
    public double omegaRadiansPerSecond;

    public ChassisSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        this.vxMetersPerSecond = vxMetersPerSecond;
        this.vyMetersPerSecond = vyMetersPerSecond;
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    public ChassisSpeeds() {
        this(0, 0, 0);
    }

    public static ChassisSpeeds fromFieldRelativeSpeeds(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            Rotation2d robotAngle) {
        return new ChassisSpeeds(
                vxMetersPerSecond * robotAngle.cos() + vyMetersPerSecond * robotAngle.sin(),
                -vxMetersPerSecond * robotAngle.sin() + vyMetersPerSecond * robotAngle.cos(),
                omegaRadiansPerSecond);
    }

    @Override
    public String toString() {
        return String.format(
                java.util.Locale.US,
                "ChassisSpeeds(Vx: %.2f m/s, Vy: %.2f m/s, Omega: %.2f rad/s)",
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond);
    }
}
