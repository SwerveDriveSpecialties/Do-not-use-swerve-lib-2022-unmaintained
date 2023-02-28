package com.swervedrivespecialties.swervelib;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    void resetAngle();

    MotorController getMotorController();

    RelativeEncoder getMotorEncoder();

    AbsoluteEncoder getAbsoluteEncoder();

    SimDeviceSim getSimulatedMotor();
}
