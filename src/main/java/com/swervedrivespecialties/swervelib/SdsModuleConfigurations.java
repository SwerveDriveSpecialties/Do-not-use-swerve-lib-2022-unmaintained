package com.swervedrivespecialties.swervelib;

public final class SdsModuleConfigurations {
    public static final ModuleConfiguration MK3_STANDARD = new ModuleConfiguration(
            "SDS-Mk3S",
            0.1016,
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            false
    );
    public static final ModuleConfiguration MK3_FAST = new ModuleConfiguration(
            "SDS-Mk3F",
            0.1016,
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 60.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            false
    );

    private SdsModuleConfigurations() {
    }
}
