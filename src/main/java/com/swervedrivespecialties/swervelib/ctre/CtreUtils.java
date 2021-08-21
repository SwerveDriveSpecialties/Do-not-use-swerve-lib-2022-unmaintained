package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.ErrorCode;

public final class CtreUtils {
    private CtreUtils() {
    }

    public static void checkCtreError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            throw new RuntimeException(String.format("%s: %s", message, errorCode.toString()));
        }
    }
}
