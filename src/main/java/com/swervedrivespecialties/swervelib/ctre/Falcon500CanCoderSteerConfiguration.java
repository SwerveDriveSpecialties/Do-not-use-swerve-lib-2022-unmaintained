package com.swervedrivespecialties.swervelib.ctre;

import java.util.Objects;

public final class Falcon500CanCoderSteerConfiguration extends Falcon500SteerConfigurationBase {
    private final int encoderPort;
    private final double encoderOffset;

    public Falcon500CanCoderSteerConfiguration(int motorPort, int encoderPort, double encoderOffset) {
        super(motorPort);
        this.encoderPort = encoderPort;
        this.encoderOffset = encoderOffset;
    }

    public int getEncoderPort() {
        return encoderPort;
    }

    public double getEncoderOffset() {
        return encoderOffset;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;
        Falcon500CanCoderSteerConfiguration that = (Falcon500CanCoderSteerConfiguration) o;
        return getEncoderPort() == that.getEncoderPort() && Double.compare(that.encoderOffset, encoderOffset) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(super.hashCode(), getEncoderPort(), encoderOffset);
    }

    @Override
    public String toString() {
        return "CanCoderConfiguration{" +
                "motorPort=" + getMotorPort() +
                ", encoderPort=" + getEncoderPort() +
                ", encoderOffset=" + Math.toDegrees(getEncoderOffset()) +
                "}";
    }
}
