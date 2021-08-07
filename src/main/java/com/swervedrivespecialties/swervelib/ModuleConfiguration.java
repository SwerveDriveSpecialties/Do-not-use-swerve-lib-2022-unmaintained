package com.swervedrivespecialties.swervelib;

import java.util.Arrays;
import java.util.Objects;

public class ModuleConfiguration {
    private final String modelIdentifier;

    private final double wheelDiameter;
    private final double[] driveReductions;

    private final double[] steerReductions;

    public ModuleConfiguration(String modelIdentifier, double wheelDiameter, double[] driveReductions, double[] steerReductions) {
        this.modelIdentifier = modelIdentifier;
        this.wheelDiameter = wheelDiameter;
        this.driveReductions = driveReductions;
        this.steerReductions = steerReductions;
    }

    public String getModelIdentifier() {
        return modelIdentifier;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public double[] getDriveReductions() {
        return driveReductions;
    }

    public double getOverallDriveReduction() {
        return Arrays.stream(getDriveReductions()).reduce(1.0, (a, b) -> a * b);
    }

    public double[] getSteerReductions() {
        return steerReductions;
    }

    public double getOverallSteerReduction() {
        return Arrays.stream(getSteerReductions()).reduce(1.0, (a, b) -> a * b);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        ModuleConfiguration that = (ModuleConfiguration) o;
        return Double.compare(that.wheelDiameter, wheelDiameter) == 0 && modelIdentifier.equals(that.modelIdentifier) && Arrays.equals(driveReductions, that.driveReductions) && Arrays.equals(steerReductions, that.steerReductions);
    }

    @Override
    public int hashCode() {
        int result = Objects.hash(modelIdentifier, wheelDiameter);
        result = 31 * result + Arrays.hashCode(driveReductions);
        result = 31 * result + Arrays.hashCode(steerReductions);
        return result;
    }

    @Override
    public String toString() {
        return "ModuleConfiguration{" +
                "modelIdentifier='" + modelIdentifier + '\'' +
                ", wheelDiameter=" + wheelDiameter +
                ", driveReductions=" + Arrays.toString(driveReductions) +
                ", steerReductions=" + Arrays.toString(steerReductions) +
                '}';
    }
}
