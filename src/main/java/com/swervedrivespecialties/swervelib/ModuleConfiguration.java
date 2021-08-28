package com.swervedrivespecialties.swervelib;

import java.util.Objects;

public class ModuleConfiguration {
    private final String modelIdentifier;

    private final double wheelDiameter;
    private final double driveReduction;
    private final boolean driveInverted;

    private final double steerReduction;
    private final boolean steerInverted;

    public ModuleConfiguration(String modelIdentifier,
                               double wheelDiameter, double driveReduction, boolean driveInverted,
                               double steerReduction, boolean steerInverted) {
        this.modelIdentifier = Objects.requireNonNull(modelIdentifier);
        this.wheelDiameter = wheelDiameter;
        this.driveReduction = driveReduction;
        this.driveInverted = driveInverted;
        this.steerReduction = steerReduction;
        this.steerInverted = steerInverted;
    }

    public String getModelIdentifier() {
        return modelIdentifier;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public double getDriveReduction() {
        return driveReduction;
    }

    public boolean isDriveInverted() {
        return driveInverted;
    }

    public double getSteerReduction() {
        return steerReduction;
    }

    public boolean isSteerInverted() {
        return steerInverted;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        ModuleConfiguration that = (ModuleConfiguration) o;
        return Double.compare(
                that.getWheelDiameter(), getWheelDiameter()) == 0 &&
                Double.compare(that.getDriveReduction(), getDriveReduction()) == 0 &&
                isDriveInverted() == that.isDriveInverted() &&
                Double.compare(that.getSteerReduction(), getSteerReduction()) == 0 &&
                isSteerInverted() == that.isSteerInverted() &&
                getModelIdentifier().equals(that.getModelIdentifier());
    }

    @Override
    public int hashCode() {
        return Objects.hash(
                getModelIdentifier(),
                getWheelDiameter(),
                getDriveReduction(),
                isDriveInverted(),
                getSteerReduction(),
                isSteerInverted()
        );
    }

    @Override
    public String toString() {
        return "ModuleConfiguration{" +
                "modelIdentifier='" + modelIdentifier + '\'' +
                ", wheelDiameter=" + wheelDiameter +
                ", driveReduction=" + driveReduction +
                ", driveInverted=" + driveInverted +
                ", steerReduction=" + steerReduction +
                ", steerInverted=" + steerInverted +
                '}';
    }
}
