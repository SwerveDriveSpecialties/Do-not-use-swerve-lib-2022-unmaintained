package com.swervedrivespecialties.swervelib;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SwerveModuleFactory<DriveConfiguration, SteerConfiguration> {
    private final ModuleConfiguration moduleConfiguration;
    private final DriveControllerFactory<?, DriveConfiguration> driveControllerFactory;
    private final SteerControllerFactory<?, SteerConfiguration> steerControllerFactory;

    public SwerveModuleFactory(ModuleConfiguration moduleConfiguration,
                               DriveControllerFactory<?, DriveConfiguration> driveControllerFactory,
                               SteerControllerFactory<?, SteerConfiguration> steerControllerFactory) {
        this.moduleConfiguration = moduleConfiguration;
        this.driveControllerFactory = driveControllerFactory;
        this.steerControllerFactory = steerControllerFactory;
    }

    public SwerveModule create(DriveConfiguration driveConfiguration, SteerConfiguration steerConfiguration) {
        var driveController = driveControllerFactory.create(driveConfiguration, moduleConfiguration);
        var steerController = steerControllerFactory.create(steerConfiguration, moduleConfiguration);

        return new ModuleImplementation(driveController, steerController);
    }

    public SwerveModule create(ShuffleboardLayout container, DriveConfiguration driveConfiguration, SteerConfiguration steerConfiguration) {
        var driveController = driveControllerFactory.create(
                container,
                driveConfiguration,
                moduleConfiguration
        );
        var steerContainer = steerControllerFactory.create(
                container,
                steerConfiguration,
                moduleConfiguration
        );

        return new ModuleImplementation(driveController, steerContainer);
    }

    private static class ModuleImplementation implements SwerveModule {
        private final DriveController driveController;
        private final SteerController steerController;

        private double m_prevTimeSeconds = Timer.getFPGATimestamp();
        private final double m_nominalDtS = 0.02; // Seconds
        public static final double TURN_KV = 0.05;
        public static final double DRIVE_KV = 0.15;
        public static final double TURN_KS = 0.001;
        public static final double DRIVE_KS = 0.001;

        private ModuleImplementation(DriveController driveController, SteerController steerController) {
            this.driveController = driveController;
            this.steerController = steerController;
        }

        @Override
        public double getDriveVelocity() {
            return driveController.getStateVelocity();
        }

        @Override
        public double getDrivePosition() {
            return driveController.getPosition();
        }

        @Override
        public void setDrivePosition(double position) {
            driveController.setPosition(position);
        }

        @Override
        public double getSteerAngle() {
            return steerController.getStateAngle();
        }

        @Override
        public void set(double driveVoltage, double steerAngle) {
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            double difference = steerAngle - getSteerAngle();
            // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
            if (difference >= Math.PI) {
                steerAngle -= 2.0 * Math.PI;
            } else if (difference < -Math.PI) {
                steerAngle += 2.0 * Math.PI;
            }
            difference = steerAngle - getSteerAngle(); // Recalculate difference

            // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
            // movement of the module is less than 90 deg
            if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
                // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
                steerAngle += Math.PI;
                driveVoltage *= -1.0;
            }

            // Put the target angle back into the range [0, 2pi)
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            driveController.setReferenceVoltage(driveVoltage);
            steerController.setReferenceAngle(steerAngle);
        }

        @Override
        public SteerController getSteerController() {
           return steerController;
        }

        @Override
        public DriveController getDriveController() {
            return driveController;
        }

        @Override
        public void resetAngle() {
            steerController.resetAngle();
        }

        @Override
        public void simulationPeriodic() {
            double currentTimeSeconds = Timer.getFPGATimestamp();
            double dtS = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : m_nominalDtS;
            m_prevTimeSeconds = currentTimeSeconds;
            simulationPeriodic(dtS);
        }

        private void simulationPeriodic(double dtS) {
            SimDeviceSim driveSim = driveController.getSimulatedMotor();
            SimDouble driveSpeed = driveSim.getDouble("Applied Output");
            SimDouble driveVelocity = driveSim.getDouble("Velocity");
            SimDouble drivePosition = driveSim.getDouble("Position");

            SimDeviceSim turnSim = steerController.getSimulatedMotor();
            SimDouble turnSpeed = turnSim.getDouble("Applied Output");
            SimDouble turnVelocity = turnSim.getDouble("Velocity");
            SimDouble turnPosition = turnSim.getDouble("Position");

            // derive velocity from motor output
            double driveVM_s = simulatedVelocity(driveSpeed.get() / 12, DRIVE_KS, DRIVE_KV);
            double turnVRad_s = simulatedVelocity(turnSpeed.get() / 12, TURN_KS, TURN_KV);

            // set the encoders using the derived velocity
            driveVelocity.set(driveVM_s);
            turnVelocity.set(turnVRad_s); // This is me adding this
            drivePosition.set(drivePosition.get() + driveVM_s * dtS); // m_DriveEncoderSim.setDistance(m_DriveEncoderSim.getDistance() + driveVM_s * dtS);
            turnPosition.set(turnPosition.get() + turnVRad_s * dtS);
    
        }

        public double simulatedVelocity(double output, double ks, double kv) {
            // Invert feedforward.
            double result = (output - ks * Math.signum(output)) / kv;
            // Add low-frequency noise.
            // result += 0.1 * pinkNoise.nextValue();
            // Add inertia.
            return 0.5 * getDriveVelocity() + 0.5 * result; // m_driveEncoder.getRate()
        }
    }
}
