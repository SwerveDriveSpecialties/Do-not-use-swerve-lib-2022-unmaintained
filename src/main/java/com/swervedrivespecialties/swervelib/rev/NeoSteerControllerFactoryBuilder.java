package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.*;
import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class NeoSteerControllerFactoryBuilder {
    // PID configuration
    private double pidProportional = Double.NaN;
    private double pidIntegral = Double.NaN;
    private double pidDerivative = Double.NaN;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public NeoSteerControllerFactoryBuilder withPidConstants(double proportional, double integral, double derivative) {
        this.pidProportional = proportional;
        this.pidIntegral = integral;
        this.pidDerivative = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(pidProportional) && Double.isFinite(pidIntegral) && Double.isFinite(pidDerivative);
    }

    public NeoSteerControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public NeoSteerControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public <T> SteerControllerFactory<ControllerImplementation, NeoSteerConfiguration<T>> build(AbsoluteEncoderFactory<T> encoderFactory) {
        return new FactoryImplementation<>(encoderFactory);
    }

    public class FactoryImplementation<T> implements SteerControllerFactory<ControllerImplementation, NeoSteerConfiguration<T>> {
        private final AbsoluteEncoderFactory<T> encoderFactory;

        public FactoryImplementation(AbsoluteEncoderFactory<T> encoderFactory) {
            this.encoderFactory = encoderFactory;
        }

        @Override
        public void addDashboardEntries(ShuffleboardContainer container, ControllerImplementation controller) {
            SteerControllerFactory.super.addDashboardEntries(container, controller);
            container.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(controller.absoluteEncoder.getAbsoluteAngle()));
        }

        @Override
        public ControllerImplementation create(NeoSteerConfiguration<T> steerConfiguration, ModuleConfiguration moduleConfiguration) {
            AbsoluteEncoder absoluteEncoder = encoderFactory.create(steerConfiguration.getEncoderConfiguration());

            CANSparkMax motor = new CANSparkMax(steerConfiguration.getMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);
            RevUtils.checkNeoError(motor.setIdleMode(CANSparkMax.IdleMode.kBrake), "Failed to set NEO idle mode");
            motor.setInverted(moduleConfiguration.getSteerReductions().length % 2 == 0);
            if (hasVoltageCompensation()) {
                RevUtils.checkNeoError(motor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");
            }
            if (hasCurrentLimit()) {
                RevUtils.checkNeoError(motor.setSmartCurrentLimit((int) Math.round(currentLimit)), "Failed to set NEO current limits");
            }

            CANEncoder integratedEncoder = motor.getEncoder();
            RevUtils.checkNeoError(integratedEncoder.setPositionConversionFactor(2.0 * Math.PI * moduleConfiguration.getOverallSteerReduction()), "Failed to set NEO encoder conversion factor");
            RevUtils.checkNeoError(integratedEncoder.setPosition(absoluteEncoder.getAbsoluteAngle()), "Failed to set NEO encoder position");

            CANPIDController controller = motor.getPIDController();
            if (hasPidConstants()) {
                RevUtils.checkNeoError(controller.setP(pidProportional), "Failed to set NEO PID proportional constant");
                RevUtils.checkNeoError(controller.setI(pidIntegral), "Failed to set NEO PID integral constant");
                RevUtils.checkNeoError(controller.setD(pidDerivative), "Failed to set NEO PID derivative constant");
            }
            RevUtils.checkNeoError(controller.setFeedbackDevice(integratedEncoder), "Failed to set NEO PID feedback device");

            return new ControllerImplementation(motor, absoluteEncoder);
        }
    }

    public static class ControllerImplementation implements SteerController {
        @SuppressWarnings({"FieldCanBeLocal", "unused"})
        private final CANSparkMax motor;
        private final CANPIDController controller;
        private final CANEncoder motorEncoder;
        private final AbsoluteEncoder absoluteEncoder;

        private double referenceAngleRadians = 0;

        public ControllerImplementation(CANSparkMax motor, AbsoluteEncoder absoluteEncoder) {
            this.motor = motor;
            this.controller = motor.getPIDController();
            this.motorEncoder = motor.getEncoder();
            this.absoluteEncoder = absoluteEncoder;
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            double currentAngleRadians = motorEncoder.getPosition();
            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }

            // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
            double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
            if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
                adjustedReferenceAngleRadians -= 2.0 * Math.PI;
            } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
                adjustedReferenceAngleRadians += 2.0 * Math.PI;
            }

            this.referenceAngleRadians = referenceAngleRadians;

            controller.setReference(adjustedReferenceAngleRadians, ControlType.kPosition);
        }

        @Override
        public double getStateAngle() {
            double motorAngleRadians = motorEncoder.getPosition();
            motorAngleRadians %= 2.0 * Math.PI;
            if (motorAngleRadians < 0.0) {
                motorAngleRadians += 2.0 * Math.PI;
            }

            return motorAngleRadians;
        }
    }
}
