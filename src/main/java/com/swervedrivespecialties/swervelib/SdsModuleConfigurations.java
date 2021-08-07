package com.swervedrivespecialties.swervelib;

public final class SdsModuleConfigurations {
    public static final ModuleConfiguration MK3_STANDARD = new ModuleConfiguration(
            "SDS-Mk3S",
            4.0,
            new double[]{14.0 / 50.0, 28.0 / 16.0, 15.0 / 60.0},
            new double[]{1.0 / 12.8}
    );
    public static final ModuleConfiguration MK3_FAST = new ModuleConfiguration(
            "SDS-Mk3F",
            4.0,
            new double[]{16.0 / 48.0, 28.0 / 16.0, 15.0 / 60.0},
            new double[]{1.0 / 12.8}
    );

    private SdsModuleConfigurations() {
    }
}
