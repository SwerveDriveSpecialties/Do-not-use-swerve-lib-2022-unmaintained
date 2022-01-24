package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.REVLibError;

public final class RevUtils {
    private RevUtils() {}

    public static void checkNeoError(REVLibError error, String message) {
        if (error != REVLibError.kOk) {
            throw new RuntimeException(String.format("%s: %s", message, error.toString()));
        }
    }
}
