package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class Mk3SwerveModuleFactory {
    private static final double DEFAULT_WHEEL_DIAMETER = 0.1016; // 4 in

    @SuppressWarnings("PointlessArithmeticExpression")
    private static final double STEER_REDUCTION = 12.8 / 1.0;

    private static final double FALCON_STEER_KV = 0.272832;
    private static final double FALCON_STEER_KA = 0.0133308;
    private static final double FALCON_STEER_KS = 0.048979;

    private static final double FALCON_STEER_P = 0.4;
    private static final double FALCON_STEER_I = 0.0;
    private static final double FALCON_STEER_D = 0.25;


    private MotorType driveMotorType = null;
    private double driveReduction = Double.NaN;
    private double driveWheelDiameter = DEFAULT_WHEEL_DIAMETER;

    private MotorType steerMotorType = null;

    public Mk3SwerveModuleFactory() {
    }

    public Mk3SwerveModuleFactory(
            MotorType driveMotorType,
            DriveType driveType,
            MotorType steerMotorType
    ) {
        configDrive(driveMotorType, driveType);
        configSteer(steerMotorType);
    }

    private void checkFullyConfigured() {
        if (driveMotorType == null || !Double.isFinite(driveReduction)) {
            throw new IllegalStateException("Module drive is not configured. Configure it by calling 'Mk3SwerveModuleFactory.configDrive'");
        }

        if (steerMotorType == null) {
            throw new IllegalStateException("Module steer is not configured. Configure it by calling 'Mk3SwerveModuleFactory.configSteer'");
        }
    }

    private void setupDriveMotor(TalonFX driveMotor) {
        // TODO: Implement
//        throw new UnsupportedOperationException("Not yet implemented");
    }

    private void setupDriveMotor(CANSparkMax driveMotor) {
        // TODO: Implement
        throw new UnsupportedOperationException("Not yet implemented");
    }

    private void setupSteerMotor(TalonFX steerMotor) {
        final double sensorPositionCoefficient = 2.0 * Math.PI / 2048.0 / STEER_REDUCTION;
        final double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 11.0;
        config.slot0.kP = FALCON_STEER_P;
        config.slot0.kI = FALCON_STEER_I;
        config.slot0.kD = FALCON_STEER_D;
        config.slot0.kF = (1023.0 * sensorVelocityCoefficient / 12.0) * FALCON_STEER_KV;
        config.motionCruiseVelocity = 2.0 / FALCON_STEER_KV / sensorVelocityCoefficient;
        config.motionAcceleration = (8.0 - 2.0) / FALCON_STEER_KA / sensorVelocityCoefficient;

        steerMotor.configAllSettings(config);
        steerMotor.enableVoltageCompensation(true);
        steerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
    }

    private void setupSteerMotor(CANSparkMax driveMotor) {
        // TODO: Implement
        throw new UnsupportedOperationException("Not yet implemented");
    }

    private void setupSteerEncoder(CANCoder encoder, double encoderOffset) {
        // TODO: Implement
//        throw new UnsupportedOperationException("Not yet implemented");
    }

    public void configDrive(MotorType motorType, DriveType driveType) {
        configDrive(motorType, driveType, DEFAULT_WHEEL_DIAMETER);
    }

    public void configDrive(MotorType motorType, DriveType driveType, double wheelDiameter) {
        configDrive(motorType, driveType.getReduction(), wheelDiameter);
    }

    public void configDrive(MotorType motorType, double reduction, double wheelDiameter) {
        driveMotorType = motorType;
        driveReduction = reduction;
        driveWheelDiameter = wheelDiameter;
    }

    public void configSteer(MotorType motorType) {
        steerMotorType = motorType;
    }

    public SwerveModule create(TalonFX driveMotor, TalonFX steerMotor,
                               CANCoder steerEncoder, double steerEncoderOffset) {
        checkFullyConfigured();

        setupDriveMotor(driveMotor);
        setupSteerMotor(steerMotor);
        setupSteerEncoder(steerEncoder, steerEncoderOffset);

        return new Implementation(
                () -> driveMotor.getSelectedSensorVelocity(), // TODO: Convert units
                () -> steerMotor.getSelectedSensorPosition(), // TODO: Convert units
                percentOutput -> driveMotor.set(TalonFXControlMode.PercentOutput, percentOutput),
                targetAngle -> steerMotor.set(TalonFXControlMode.MotionMagic, targetAngle) // TODO: Convert units
        );
    }

    private static class Implementation extends SwerveModuleBase {
        private final DoubleSupplier driveVelocitySupplier;
        private final DoubleSupplier steerAngleSupplier;

        private final DoubleConsumer drivePercentOutputConsumer;
        private final DoubleConsumer steerTargetAngleConsumer;

        private Implementation(
                DoubleSupplier driveVelocitySupplier, DoubleSupplier steerAngleSupplier,
                DoubleConsumer drivePercentOutputConsumer, DoubleConsumer steerTargetAngleConsumer
        ) {
            this.driveVelocitySupplier = driveVelocitySupplier;
            this.steerAngleSupplier = steerAngleSupplier;
            this.drivePercentOutputConsumer = drivePercentOutputConsumer;
            this.steerTargetAngleConsumer = steerTargetAngleConsumer;
        }

        @Override
        public double getDriveVelocity() {
            return driveVelocitySupplier.getAsDouble();
        }

        @Override
        public double getSteerAngle() {
            return steerAngleSupplier.getAsDouble();
        }

        @Override
        protected void update(double drivePercentOutput, double targetSteerAngle) {
            drivePercentOutputConsumer.accept(drivePercentOutput);
            steerTargetAngleConsumer.accept(targetSteerAngle);
        }
    }

    public enum MotorType {
        FALCON_500,
        NEO
    }

    public enum DriveType {
        STANDARD,
        FAST;

        @SuppressWarnings("PointlessArithmeticExpression")
        private static final double STANDARD_DRIVE_REDUCTION = 8.16 / 1.0;
        @SuppressWarnings("PointlessArithmeticExpression")
        private static final double FAST_DRIVE_REDUCTION = 6.86 / 1.0;

        public double getReduction() {
            switch (this) {
                case STANDARD:
                    return STANDARD_DRIVE_REDUCTION;
                case FAST:
                    return FAST_DRIVE_REDUCTION;
                default:
                    throw new UnsupportedOperationException("Unknown drive type " + this);
            }
        }
    }
}
