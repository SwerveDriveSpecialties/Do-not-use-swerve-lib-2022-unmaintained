package com.swervedrivespecialties.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getDrivePosition();

    void setDrivePosition(double position);

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void resetAngle();

    SteerController getSteerController();

    DriveController getDriveController();

    void simulationPeriodic();
}
