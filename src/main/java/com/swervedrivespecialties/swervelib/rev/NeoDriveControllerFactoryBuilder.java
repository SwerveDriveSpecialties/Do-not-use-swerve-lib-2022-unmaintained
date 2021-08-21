package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;

public final class NeoDriveControllerFactoryBuilder {
    private double nominalVoltage = Double.NaN;

    public NeoDriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer id, ModuleConfiguration moduleConfiguration) {
            CANSparkMax motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
            motor.setInverted(moduleConfiguration.getDriveReductions().length % 2 == 0);

            // TODO: Configure builtin encoder
            // TODO: Configure CAN frame rates

            // Setup voltage compensation
            if (hasVoltageCompensation()) {
                CANError error = motor.enableVoltageCompensation(nominalVoltage);
                if (error != CANError.kOk) {
                    // Failed to enable voltage compensation
                    throw new RuntimeException("Failed to enable voltage compensation for NEO");
                }
            }

            // Set neutral mode to brake
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

            return new ControllerImplementation(motor, motor.getEncoder());
        }
    }

    private static class ControllerImplementation implements DriveController {
        private final CANSparkMax motor;
        private final CANEncoder encoder;

        private ControllerImplementation(CANSparkMax motor, CANEncoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setVoltage(voltage);
        }

        @Override
        public double getStateVelocity() {
            return encoder.getVelocity();
        }
    }
}
