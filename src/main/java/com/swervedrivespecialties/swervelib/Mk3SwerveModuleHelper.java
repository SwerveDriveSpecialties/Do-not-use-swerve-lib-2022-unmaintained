package com.swervedrivespecialties.swervelib;

import com.swervedrivespecialties.swervelib.ctre.CanCoderFactoryBuilder;
import com.swervedrivespecialties.swervelib.ctre.Falcon500CanCoderSteerConfiguration;
import com.swervedrivespecialties.swervelib.ctre.Falcon500DriveControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.ctre.Falcon500SteerControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.rev.NeoDriveControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.rev.NeoSteerConfiguration;
import com.swervedrivespecialties.swervelib.rev.NeoSteerControllerFactoryBuilder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public final class Mk3SwerveModuleHelper {
    private Mk3SwerveModuleHelper() {
    }

    private static SwerveModuleFactory<Integer, Falcon500CanCoderSteerConfiguration> getFalcon500Factory(GearRatio gearRatio) {
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                new Falcon500DriveControllerFactoryBuilder()
                        .withVoltageCompensation(12.0)
                        .build(),
                new Falcon500SteerControllerFactoryBuilder()
                        .withVoltageCompensation(12.0)
                        .withPidConstants(0.1, 0.0, 0.0)
//                        .withMotionMagic(0.272832, 0.0133308, 0.048979)
                        .buildWithCanCoder()
        );
    }

    private static SwerveModuleFactory<Integer, NeoSteerConfiguration<CanCoderFactoryBuilder.AbsoluteConfiguration>> getNeoFactory(GearRatio gearRatio) {
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                new NeoDriveControllerFactoryBuilder()
                        .withVoltageCompensation(12.0)
                        .build(),
                new NeoSteerControllerFactoryBuilder()
                        .withVoltageCompensation(12.0)
                        .withPidConstants(1.0, 0.0, 0.1)
                        .withCurrentLimit(20.0)
                        .build(new CanCoderFactoryBuilder().build())
        );
    }

    /**
     * Creates a Mk3 swerve module that uses Falcon 500s for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset
    ) {
        var factory = getFalcon500Factory(gearRatio);

        return factory.create(
                container,
                driveMotorPort,
                new Falcon500CanCoderSteerConfiguration(steerMotorPort, steerEncoderPort, steerOffset)
        );
    }

    /**
     * Creates a Mk3 swerve module that uses a Falcon 500 for driving and steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset
    ) {
        var factory = getFalcon500Factory(gearRatio);

        return factory.create(
                driveMotorPort,
                new Falcon500CanCoderSteerConfiguration(steerMotorPort, steerEncoderPort, steerOffset));
    }

    public static SwerveModule createNeo(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset
    ) {
        var factory = getNeoFactory(gearRatio);

        return factory.create(
                container,
                driveMotorPort,
                new NeoSteerConfiguration<>(steerMotorPort, new CanCoderFactoryBuilder.AbsoluteConfiguration(steerEncoderPort, steerOffset))
        );
    }

    public static SwerveModule createNeo(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset
    ) {
        var factory = getNeoFactory(gearRatio);

        return factory.create(
                driveMotorPort,
                new NeoSteerConfiguration<>(steerMotorPort, new CanCoderFactoryBuilder.AbsoluteConfiguration(steerEncoderPort, steerOffset))
        );
    }

    public enum GearRatio {
        STANDARD(SdsModuleConfigurations.MK3_STANDARD),
        FAST(SdsModuleConfigurations.MK3_FAST);

        private final ModuleConfiguration configuration;

        GearRatio(ModuleConfiguration configuration) {
            this.configuration = configuration;
        }

        public ModuleConfiguration getConfiguration() {
            return configuration;
        }
    }
}
