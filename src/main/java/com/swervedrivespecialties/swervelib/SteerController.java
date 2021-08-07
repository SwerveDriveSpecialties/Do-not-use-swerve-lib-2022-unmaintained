package com.swervedrivespecialties.swervelib;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double steerAngle);

    double getStateAngle();
}
