package com.swervedrivespecialties.swervelib.ctre;


import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.VoltageConfigs;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;

public final class Falcon500DriveControllerFactoryBuilder {
    private static final double TICKS_PER_ROTATION = 2048.0;

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public Falcon500DriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    public Falcon500DriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }
    //LIBRARY DOCS
    //https://pro.docs.ctr-electronics.com/en/stable/index.html
    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer driveConfiguration, ModuleConfiguration moduleConfiguration) {
            var voltageConfig = new VoltageConfigs();
            var currentConfig = new CurrentLimitsConfigs();
            var motorOutputConfig = new MotorOutputConfigs();

            motorOutputConfig.NeutralMode = NeutralModeValue.Brake;
            motorOutputConfig.Inverted = moduleConfiguration.isDriveInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;


            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
            double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            if (hasVoltageCompensation()) {
                voltageConfig.PeakForwardVoltage = nominalVoltage;
                voltageConfig.PeakReverseVoltage = nominalVoltage;
            }

            if (hasCurrentLimit()) {
                currentConfig.SupplyCurrentLimit = currentLimit;
                currentConfig.SupplyCurrentLimitEnable = true;
            }

            TalonFX motor = new TalonFX(driveConfiguration, CtreUtils.kCANivoreBusName);
            var talonFXConfigurator = motor.getConfigurator();
            talonFXConfigurator.apply(voltageConfig);
            talonFXConfigurator.apply(currentConfig);

            if (hasVoltageCompensation()) {
                // Enable voltage compensation
                //motor.enableVoltageCompensation(true);
            }

            //TODO: See if this matters
            //motor.setSensorPhase(true);

            // Reduce CAN status frame rates
            // CtreUtils.checkCtreError(
            //         motor.setStatusFramePeriod(
            //                 StatusFrameEnhanced.Status_1_General,
            //                 STATUS_FRAME_GENERAL_PERIOD_MS,
            //                 CAN_TIMEOUT_MS
            //         ),
            //         "Failed to configure Falcon status frame period"
            // );

            return new ControllerImplementation(motor, sensorVelocityCoefficient);
        }
    }

    private class ControllerImplementation implements DriveController {
        private final TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double nominalVoltage = hasVoltageCompensation() ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage : 12.0;

        private ControllerImplementation(TalonFX motor, double sensorVelocityCoefficient) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setControl(new DutyCycleOut(voltage / nominalVoltage));
        }

        @Override
        public double getPosition() {
            return motor.getRotorPosition().getValue() * (sensorVelocityCoefficient / 10.0);
        }

        @Override
        public double getStateVelocity() {
            return motor.getRotorVelocity().getValue() * sensorVelocityCoefficient;
        }

        @Override
        public void setCanStatusFramePeriodReductions() {
            System.out.println("Start Falcon Drive Can Reduction.");
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);
            System.out.printf("Drive Falcon %1d: Reduced CAN message rates.", motor.getDeviceID());
            System.out.println();
        }
    }
}
