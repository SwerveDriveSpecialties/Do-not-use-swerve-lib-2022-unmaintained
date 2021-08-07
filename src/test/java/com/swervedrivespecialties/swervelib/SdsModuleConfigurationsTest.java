package com.swervedrivespecialties.swervelib;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SdsModuleConfigurationsTest {
    @Test
    void mk3StandardOverallDriveReduction() {
        assertEquals(1.0 / 8.16, SdsModuleConfigurations.MK3_STANDARD.getOverallDriveReduction(), 1e-4);
    }

    @Test
    void mk3FastOverallDriveReduction() {
        assertEquals(1.0 / 6.86, SdsModuleConfigurations.MK3_FAST.getOverallDriveReduction(), 1e-4);
    }
}
