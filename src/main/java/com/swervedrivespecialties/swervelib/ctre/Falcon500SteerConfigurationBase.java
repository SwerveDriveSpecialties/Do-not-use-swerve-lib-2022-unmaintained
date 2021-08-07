package com.swervedrivespecialties.swervelib.ctre;

import java.util.Objects;

public abstract class Falcon500SteerConfigurationBase {
    private final int motorPort;

    public Falcon500SteerConfigurationBase(int motorPort) {
        this.motorPort = motorPort;
    }

    public int getMotorPort() {
        return motorPort;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Falcon500SteerConfigurationBase that = (Falcon500SteerConfigurationBase) o;
        return getMotorPort() == that.getMotorPort();
    }

    @Override
    public int hashCode() {
        return Objects.hash(getMotorPort());
    }

    @Override
    public String toString() {
        return "ConfigurationBase{" +
                "motorPort=" + motorPort +
                '}';
    }
}
