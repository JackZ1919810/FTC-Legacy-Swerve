package org.firstinspires.ftc.teamcode.util;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;

    private double setpoint;
    private double errorSum;
    private double lastError;
    private double lastTime;

    private boolean continuous;
    private double minInput;
    private double maxInput;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void enableContinuousInput(double min, double max) {
        this.continuous = true;
        this.minInput = min;
        this.maxInput = max;
    }

    public double calculate(double measurement) {
        double currentTime = System.currentTimeMillis() / 1000.0;
        if (lastTime == 0) lastTime = currentTime;
        double dt = currentTime - lastTime;

        double error = setpoint - measurement;

        if (continuous) {
            double range = maxInput - minInput;
            error %= range;
            if (Math.abs(error) > range / 2.0) {
                error -= Math.copySign(range, error);
            }
        }

        errorSum += error * dt;
        double errorRate = dt > 0 ? (error - lastError) / dt : 0;

        lastError = error;
        lastTime = currentTime;

        return kP * error + kI * errorSum + kD * errorRate;
    }

    public void reset() {
        errorSum = 0;
        lastError = 0;
        lastTime = 0;
    }

    public void setP(double kP) { this.kP = kP; }
    public void setI(double kI) { this.kI = kI; }
    public void setD(double kD) { this.kD = kD; }
}
