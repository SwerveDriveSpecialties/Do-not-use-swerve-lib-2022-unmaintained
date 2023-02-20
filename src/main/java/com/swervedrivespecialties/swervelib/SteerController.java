package com.swervedrivespecialties.swervelib;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    void resetAngle();

    MotorController getMotorController();

    RelativeEncoder getMotorEncoder();

    AbsoluteEncoder getAbsoluteEncoder();
}
