package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.swervedrivespecialties.swervelib.*;

import static com.swervedrivespecialties.swervelib.ctre.CtreUtils.checkCtreError;

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
    private double currentLimit = Double.NaN;

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

    public Falcon500SteerControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public <T> SteerControllerFactory<ControllerImplementation, Falcon500SteerConfiguration<T>> build(AbsoluteEncoderFactory<T> absoluteEncoderFactory) {
        return new FactoryImplementation<>(absoluteEncoderFactory);
    }

    private class FactoryImplementation<T> implements SteerControllerFactory<ControllerImplementation, Falcon500SteerConfiguration<T>> {
        private final AbsoluteEncoderFactory<T> encoderFactory;

        private FactoryImplementation(AbsoluteEncoderFactory<T> encoderFactory) {
            this.encoderFactory = encoderFactory;
        }

        @Override
        public ControllerImplementation create(Falcon500SteerConfiguration<T> steerConfiguration, ModuleConfiguration moduleConfiguration) {
            AbsoluteEncoder absoluteEncoder = encoderFactory.create(steerConfiguration.getEncoderConfiguration());

            final double sensorPositionCoefficient = 2.0 * Math.PI / TICKS_PER_ROTATION * moduleConfiguration.getSteerReduction();
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
            if (hasCurrentLimit()) {
                motorConfiguration.supplyCurrLimit.currentLimit = currentLimit;
                motorConfiguration.supplyCurrLimit.enable = true;
            }

            // TODO: Current limiting

            TalonFX motor = new TalonFX(steerConfiguration.getMotorPort());
            checkCtreError(motor.configAllSettings(motorConfiguration, 250), "Failed to configure Falcon 500 settings");

            if (hasVoltageCompensation()) {
                motor.enableVoltageCompensation(true);
            }
            checkCtreError(motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 250), "Failed to set Falcon 500 feedback sensor");
            motor.setSensorPhase(moduleConfiguration.isSteerInverted());
            motor.setNeutralMode(NeutralMode.Brake);

            checkCtreError(motor.setSelectedSensorPosition(absoluteEncoder.getAbsoluteAngle() / sensorPositionCoefficient, 0, 250), "Failed to set Falcon 500 encoder position");

            return new ControllerImplementation(motor, sensorPositionCoefficient, hasMotionMagic() ? TalonFXControlMode.MotionMagic : TalonFXControlMode.Position, absoluteEncoder);
        }
    }

    private static class ControllerImplementation implements SteerController {
        private final TalonFX motor;
        private final double motorEncoderPositionCoefficient;
        private final TalonFXControlMode motorControlMode;
        @SuppressWarnings({"unused", "FieldCanBeLocal"})
        private final AbsoluteEncoder absoluteEncoder;

        private double referenceAngleRadians = 0.0;

        private ControllerImplementation(TalonFX motor,
                                         double motorEncoderPositionCoefficient,
                                         TalonFXControlMode motorControlMode,
                                         AbsoluteEncoder absoluteEncoder) {
            this.motor = motor;
            this.motorEncoderPositionCoefficient = motorEncoderPositionCoefficient;
            this.motorControlMode = motorControlMode;
            this.absoluteEncoder = absoluteEncoder;
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            double currentAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;
            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }

            // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
            double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
            if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
                adjustedReferenceAngleRadians -= 2.0 * Math.PI;
            } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
                adjustedReferenceAngleRadians += 2.0 * Math.PI;
            }

            motor.set(motorControlMode, adjustedReferenceAngleRadians / motorEncoderPositionCoefficient);

            this.referenceAngleRadians = referenceAngleRadians;
        }

        @Override
        public double getStateAngle() {
            double motorAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;
            motorAngleRadians %= 2.0 * Math.PI;
            if (motorAngleRadians < 0.0) {
                motorAngleRadians += 2.0 * Math.PI;
            }

            return motorAngleRadians;
        }
    }
}
