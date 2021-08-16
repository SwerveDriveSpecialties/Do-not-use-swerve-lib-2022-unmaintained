package com.swervedrivespecialties.swervelib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

@FunctionalInterface
public interface DriveControllerFactory<DriveConfiguration> {
    DriveController create(DriveConfiguration driveConfiguration, ModuleConfiguration moduleConfiguration);

    default DriveController create(
            ShuffleboardContainer container,
            DriveConfiguration driveConfiguration,
            ModuleConfiguration moduleConfiguration
    ) {
        var controller = create(driveConfiguration, moduleConfiguration);
        container.addNumber("Current Velocity", controller::getStateVelocity);

        return controller;
    }
}
