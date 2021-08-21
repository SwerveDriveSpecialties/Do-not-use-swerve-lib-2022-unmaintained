package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.CANError;

public final class RevUtils {
    private RevUtils() {}

    public static void checkNeoError(CANError error, String message) {
        if (error != CANError.kOk) {
            throw new RuntimeException(String.format("%s: %s", message, error.toString()));
        }
    }
}
