package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AnalogEncoder {
    private final AnalogInput encoder;
    private double offset;

    public AnalogEncoder(HardwareMap hMap, String deviceName) {
        this.encoder = hMap.get(AnalogInput.class, deviceName);
        this.offset = 0;
    }

    public AnalogEncoder(HardwareMap hMap, String deviceName, double offset) {
        this.encoder = hMap.get(AnalogInput.class, deviceName);
        this.offset = offset;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public Rotation2d getRotation() {
        double rawRadians = (encoder.getVoltage() / encoder.getMaxVoltage()) * 2.0 * Math.PI;
        return new Rotation2d(rawRadians - offset);
    }

    public double getVoltage() {
        return encoder.getVoltage();
    }

    public double getMaxVoltage() {
        return encoder.getMaxVoltage();
    }
}
