package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;
import org.ejml.simple.SimpleMatrix;

/**
 * Helper class that converts a chassis velocity (dx, dy, and dtheta components) into individual
 * module states (speed and angle).
 */
public class SwerveDriveKinematics {
    private final SimpleMatrix m_inverseKinematics;
    private final SimpleMatrix m_forwardKinematics;

    private final int m_numModules;
    private final Translation2d[] m_modules;
    private Rotation2d[] m_moduleHeadings;
    private Translation2d m_prevCoR = Translation2d.kZero;

    /**
     * Constructs a swerve drive kinematics object.
     *
     * @param moduleTranslationsMeters The locations of the modules relative to the physical center of
     *     the robot.
     */
    public SwerveDriveKinematics(Translation2d... moduleTranslationsMeters) {
        if (moduleTranslationsMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
        }
        m_numModules = moduleTranslationsMeters.length;
        m_modules = Arrays.copyOf(moduleTranslationsMeters, m_numModules);
        m_moduleHeadings = new Rotation2d[m_numModules];
        Arrays.fill(m_moduleHeadings, Rotation2d.kZero);
        m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);

        for (int i = 0; i < m_numModules; i++) {
            m_inverseKinematics.set(i * 2 + 0, 0, 1);
            m_inverseKinematics.set(i * 2 + 0, 1, 0);
            m_inverseKinematics.set(i * 2 + 0, 2, -m_modules[i].getY());

            m_inverseKinematics.set(i * 2 + 1, 0, 0);
            m_inverseKinematics.set(i * 2 + 1, 1, 1);
            m_inverseKinematics.set(i * 2 + 1, 2, m_modules[i].getX());
        }
        m_forwardKinematics = m_inverseKinematics.pseudoInverse();
    }

    /**
     * Reset the internal swerve module headings.
     */
    public void resetHeadings(Rotation2d... moduleHeadings) {
        if (moduleHeadings.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of headings is not consistent with number of module locations provided in constructor");
        }
        m_moduleHeadings = Arrays.copyOf(moduleHeadings, m_numModules);
    }

    /**
     * Performs inverse kinematics to return the module states from a desired chassis velocity.
     */
    public SwerveModuleState[] toSwerveModuleStates(
            ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
        SwerveModuleState[] moduleStates = new SwerveModuleState[m_numModules];

        if (chassisSpeeds.vxMetersPerSecond == 0.0
                && chassisSpeeds.vyMetersPerSecond == 0.0
                && chassisSpeeds.omegaRadiansPerSecond == 0.0) {
            for (int i = 0; i < m_numModules; i++) {
                moduleStates[i] = new SwerveModuleState(0.0, m_moduleHeadings[i]);
            }
            return moduleStates;
        }

        if (!centerOfRotationMeters.equals(m_prevCoR)) {
            for (int i = 0; i < m_numModules; i++) {
                m_inverseKinematics.set(i * 2 + 0, 0, 1);
                m_inverseKinematics.set(i * 2 + 0, 1, 0);
                m_inverseKinematics.set(i * 2 + 0, 2, -m_modules[i].getY() + centerOfRotationMeters.getY());

                m_inverseKinematics.set(i * 2 + 1, 0, 0);
                m_inverseKinematics.set(i * 2 + 1, 1, 1);
                m_inverseKinematics.set(i * 2 + 1, 2, m_modules[i].getX() - centerOfRotationMeters.getX());
            }
            m_prevCoR = centerOfRotationMeters;
        }

        SimpleMatrix chassisSpeedsVector = new SimpleMatrix(3, 1);
        chassisSpeedsVector.set(0, 0, chassisSpeeds.vxMetersPerSecond);
        chassisSpeedsVector.set(1, 0, chassisSpeeds.vyMetersPerSecond);
        chassisSpeedsVector.set(2, 0, chassisSpeeds.omegaRadiansPerSecond);

        SimpleMatrix moduleStatesMatrix = m_inverseKinematics.mult(chassisSpeedsVector);

        for (int i = 0; i < m_numModules; i++) {
            double x = moduleStatesMatrix.get(i * 2, 0);
            double y = moduleStatesMatrix.get(i * 2 + 1, 0);

            double speed = Math.hypot(x, y);
            Rotation2d angle = speed > 1e-6 ? new Rotation2d(Math.atan2(y, x)) : m_moduleHeadings[i];

            moduleStates[i] = new SwerveModuleState(speed, angle);
            m_moduleHeadings[i] = angle;
        }

        return moduleStates;
    }

    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        return toSwerveModuleStates(chassisSpeeds, Translation2d.kZero);
    }

    /**
     * Performs forward kinematics to return the resulting chassis state from the given module states.
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... moduleStates) {
        if (moduleStates.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of module locations provided in constructor");
        }
        SimpleMatrix moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            SwerveModuleState module = moduleStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.cos());
            moduleStatesMatrix.set(i * 2 + 1, 0, module.speedMetersPerSecond * module.angle.sin());
        }

        SimpleMatrix chassisSpeedsVector = m_forwardKinematics.mult(moduleStatesMatrix);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }

    /**
     * Performs forward kinematics to return the resulting Twist2d from the given module deltas.
     */
    public Twist2d toTwist2d(SwerveModulePosition... moduleDeltas) {
        if (moduleDeltas.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of module locations provided in constructor");
        }
        SimpleMatrix moduleDeltaMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            SwerveModulePosition module = moduleDeltas[i];
            moduleDeltaMatrix.set(i * 2, 0, module.distanceMeters * module.angle.cos());
            moduleDeltaMatrix.set(i * 2 + 1, 0, module.distanceMeters * module.angle.sin());
        }

        SimpleMatrix chassisDeltaVector = m_forwardKinematics.mult(moduleDeltaMatrix);
        return new Twist2d(
                chassisDeltaVector.get(0, 0), chassisDeltaVector.get(1, 0), chassisDeltaVector.get(2, 0));
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
     */
    public static void desaturateWheelSpeeds(
            SwerveModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond) {
        double realMaxSpeed = 0;
        for (SwerveModuleState moduleState : moduleStates) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }
        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
            for (SwerveModuleState moduleState : moduleStates) {
                moduleState.speedMetersPerSecond =
                        moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
            }
        }
    }

    /**
     * Gets the locations of the modules relative to the center of rotation.
     */
    public Translation2d[] getModules() {
        return m_modules;
    }
}
