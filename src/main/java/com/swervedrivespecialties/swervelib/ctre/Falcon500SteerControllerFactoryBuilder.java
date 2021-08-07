package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SteerController;
import com.swervedrivespecialties.swervelib.SteerControllerFactory;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public final class Falcon500SteerControllerFactoryBuilder {
    private static final double TICKS_PER_ROTATION = 2048.0;

    // PID configuration
    private double proportionalConstant = Double.NaN;
    private double integralConstant = Double.NaN;
    private double derivativeConstant = Double.NaN;

    // Motion magic configuration
    private double velocityConstant = Double.NaN;
    private double accelerationConstant = Double.NaN;
    private double staticConstant = Double.NaN;

    private double nominalVoltage = Double.NaN;

    public Falcon500SteerControllerFactoryBuilder withPidConstants(double proportional, double integral, double derivative) {
        this.proportionalConstant = proportional;
        this.integralConstant = integral;
        this.derivativeConstant = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }

    public Falcon500SteerControllerFactoryBuilder withMotionMagic(double velocityConstant, double accelerationConstant, double staticConstant) {
        this.velocityConstant = velocityConstant;
        this.accelerationConstant = accelerationConstant;
        this.staticConstant = staticConstant;
        return this;
    }

    public boolean hasMotionMagic() {
        return Double.isFinite(velocityConstant) && Double.isFinite(accelerationConstant) && Double.isFinite(staticConstant);
    }

    public Falcon500SteerControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    private TalonFX createMotor(
            Falcon500SteerConfigurationBase steerConfiguration,
            ModuleConfiguration moduleConfiguration,
            double sensorPositionCoefficient
    ) {
        final double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        if (hasPidConstants()) {
            motorConfiguration.slot0.kP = proportionalConstant;
            motorConfiguration.slot0.kI = integralConstant;
            motorConfiguration.slot0.kD = derivativeConstant;
        }
        if (hasMotionMagic()) {
            if (hasVoltageCompensation()) {
                motorConfiguration.slot0.kF = (1023.0 * sensorVelocityCoefficient / nominalVoltage) * velocityConstant;
            }
            // TODO: What should be done if no nominal voltage is configured? Use a default voltage?

            // TODO: Make motion magic max voltages configurable or dynamically determine optimal values
            motorConfiguration.motionCruiseVelocity = 2.0 / velocityConstant / sensorVelocityCoefficient;
            motorConfiguration.motionAcceleration = (8.0 - 2.0) / accelerationConstant / sensorVelocityCoefficient;
        }
        if (hasVoltageCompensation()) {
            motorConfiguration.voltageCompSaturation = nominalVoltage;
        }

        // TODO: Current limiting

        TalonFX motor = new TalonFX(steerConfiguration.getMotorPort());
        ErrorCode error = motor.configAllSettings(motorConfiguration, 250);
        if (error != ErrorCode.OK) {
            throw new RuntimeException(String.format("Failed to configure Falcon 500. (Error: %s)", error));
        }

        if (hasVoltageCompensation()) {
            motor.enableVoltageCompensation(true);
        }
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 250);
        motor.setSensorPhase(moduleConfiguration.getSteerReductions().length % 2 == 0);
        motor.setNeutralMode(NeutralMode.Brake);

        return motor;
    }

    public SteerControllerFactory<ControllerImplementation, Falcon500CanCoderSteerConfiguration> buildWithCanCoder() {
        return (steerConfiguration, moduleConfiguration) -> {
            final double sensorPositionCoefficient = 2.0 * Math.PI / TICKS_PER_ROTATION * moduleConfiguration.getOverallSteerReduction();

            TalonFX motor = Falcon500SteerControllerFactoryBuilder.this.createMotor(steerConfiguration, moduleConfiguration, sensorPositionCoefficient);

            CANCoder encoder = new CANCoder(steerConfiguration.getEncoderPort());
            {
                double currentAngle = Math.toRadians(encoder.getAbsolutePosition()) + steerConfiguration.getEncoderOffset();
                motor.setSelectedSensorPosition(currentAngle / sensorPositionCoefficient);
            }

            return new ControllerImplementation(
                    referenceAngle -> {
                        double currentAngle = motor.getSelectedSensorPosition() * sensorPositionCoefficient;
                        double currentAngleMod = currentAngle % (2.0 * Math.PI);
                        if (currentAngleMod < 0.0) {
                            currentAngleMod += 2.0 * Math.PI;
                        }

                        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
                        double adjustedReferenceAngle = referenceAngle + currentAngle - currentAngleMod;
                        if (referenceAngle - currentAngleMod > Math.PI) {
                            adjustedReferenceAngle -= 2.0 * Math.PI;
                        } else if (referenceAngle - currentAngleMod < -Math.PI) {
                            adjustedReferenceAngle += 2.0 * Math.PI;
                        }

                        if (hasMotionMagic()) {
                            motor.set(TalonFXControlMode.MotionMagic, adjustedReferenceAngle / sensorPositionCoefficient);
                        } else {
                            motor.set(TalonFXControlMode.Position, adjustedReferenceAngle / sensorPositionCoefficient);
                        }
                    },
                    () -> {
//                        double encoderAngle = Math.toRadians(encoder.getAbsolutePosition()) + steerConfiguration.getEncoderOffset();
//                        encoderAngle %= 2.0 * Math.PI;
//                        if (encoderAngle < 0.0) {
//                            encoderAngle += 2.0 * Math.PI;
//                        }
                        // TODO: What should happen if the reported angles of the  encoder and the motor are out of sync?

                        double motorAngle = motor.getSelectedSensorPosition() * sensorPositionCoefficient;
                        motorAngle %= (2.0 * Math.PI);
                        if (motorAngle < 0.0) {
                            motorAngle += 2.0 * Math.PI;
                        }

                        return motorAngle;
                    }
            );
        };
    }

    private static class ControllerImplementation implements SteerController {
        private final DoubleConsumer referenceAngleConsumer;
        private final DoubleSupplier stateAngleSupplier;

        private double referenceAngle = 0.0;

        private ControllerImplementation(
                DoubleConsumer referenceAngleConsumer,
                DoubleSupplier stateAngleSupplier
        ) {
            this.referenceAngleConsumer = referenceAngleConsumer;
            this.stateAngleSupplier = stateAngleSupplier;
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngle;
        }

        @Override
        public void setReferenceAngle(double steerAngle) {
            referenceAngleConsumer.accept(steerAngle);
            this.referenceAngle = steerAngle;
        }

        @Override
        public double getStateAngle() {
            return stateAngleSupplier.getAsDouble();
        }
    }
}
