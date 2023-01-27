package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenixpro.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

public final class CtreUtils {

    public final static String kCANivoreBusName = "FRC263CANivore1";

    private CtreUtils() {
    }

    public static void checkCtreError(StatusCode errorCode, String message) {
        if (errorCode != StatusCode.OK) {
            DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
        }
    }
}
