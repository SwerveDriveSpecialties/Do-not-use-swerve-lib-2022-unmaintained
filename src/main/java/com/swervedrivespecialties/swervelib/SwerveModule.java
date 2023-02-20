package com.swervedrivespecialties.swervelib;

import com.revrobotics.RelativeEncoder;

public interface SwerveModule {
    double getDriveVelocity();

    double getDrivePosition();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void resetAngle();

    SteerController getSteerController();

    DriveController getDriveController();

}
