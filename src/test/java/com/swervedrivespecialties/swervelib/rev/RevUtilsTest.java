package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.CANError;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

class RevUtilsTest {
    @Test
    void checkNeoError() {
        assertThrows(RuntimeException.class, () -> RevUtils.checkNeoError(CANError.kError, ""));
        assertThrows(RuntimeException.class, () -> RevUtils.checkNeoError(CANError.kCantFindFirmware, ""));
        assertDoesNotThrow(() -> RevUtils.checkNeoError(CANError.kOk, ""));
    }
}
