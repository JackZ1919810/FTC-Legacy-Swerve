package org.firstinspires.ftc.teamcode.util;

public class SwerveModuleState {
    public double speedMetersPerSecond;
    public Rotation2d angle;

    public SwerveModuleState(double speedMetersPerSecond, Rotation2d angle) {
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.angle = angle;
    }

    public SwerveModuleState() {
        this(0, new Rotation2d(0));
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins.
     *
     * @param currentAngle The current angle of the module.
     * @return The optimized state.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        Rotation2d delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getRadians()) > Math.PI / 2.0) {
            return new SwerveModuleState(
                -desiredState.speedMetersPerSecond,
                desiredState.angle.plus(new Rotation2d(Math.PI))
            );
        } else {
            return desiredState;
        }
    }

    @Override
    public String toString() {
        return String.format(java.util.Locale.US, "SwerveModuleState(Speed: %.2f m/s, Angle: %s)", speedMetersPerSecond, angle);
    }
}
