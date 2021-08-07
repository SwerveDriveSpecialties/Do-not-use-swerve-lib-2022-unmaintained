package com.swervedrivespecialties.examples.mk3minibot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.swervedrivespecialties.swervelib.motion.MotionProfile;
import com.swervedrivespecialties.swervelib.motion.TrapezoidalMotionProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

import javax.annotation.Nullable;
import java.util.LinkedList;
import java.util.List;

public class RobotOld extends TimedRobot {
    private static final double DT = 5e-3;

    private static final double SENSOR_POSITION_COEFFICIENT = 2.0 * Math.PI / 2048.0 / 12.8;
    private static final double SENSOR_VELOCITY_COEFFICIENT = SENSOR_POSITION_COEFFICIENT * 10.0;

    private static final double STEER_KV = 0.022736 * 12.0;
    private static final double STEER_KA = 0.0011109 * 12.0;
    private static final double STEER_KS = 0.048979;

    private static final double STEER_MM_P = 0.5;
    private static final double STEER_MM_I = 0.0;
    private static final double STEER_MM_D = 0.25;
    private static final double STEER_MM_F = (1023.0 * SENSOR_VELOCITY_COEFFICIENT / 12.0) * STEER_KV;

    private static final double MAX_PROFILE_VELOCITY = 2.0 / STEER_KV; // 8V
    private static final double MAX_PROFILE_ACCELERATION = (4.0 - STEER_KV * MAX_PROFILE_VELOCITY) / STEER_KA;

//    private final TalonFX ssMotor = new TalonFX(1);
//    private final TalonFX pidMotor = new TalonFX(3);

    private final LinearSystem<N2, N1, N2> plant = new LinearSystem<>(
            new MatBuilder<>(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -STEER_KV / STEER_KA),
            new MatBuilder<>(Nat.N2(), Nat.N1()).fill(0.0, 1.0 / STEER_KA),
            new MatBuilder<>(Nat.N2(), Nat.N2()).fill(1.0, 0.0, 0.0, 1.0),
            new MatBuilder<>(Nat.N2(), Nat.N1()).fill(0.0, 0.0)
    );
    private final LinearQuadraticRegulator<N2, N1, N2> controller = new LinearQuadraticRegulator<>(
            plant.getA(),
            plant.getB(),
            new MatBuilder<>(Nat.N2(), Nat.N2()).fill(10.0, 0.0, 0.0, 100.0),
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01),
            DT);
    private final KalmanFilter<N2, N1, N2> observer = new KalmanFilter<>(
            Nat.N2(), Nat.N2(), plant,
            VecBuilder.fill(0.05, 0.5),
            VecBuilder.fill(0.00001, 0.01),
            DT
    );
    private final LinearSystemLoop<N2, N1, N2> loop = new LinearSystemLoop<>(plant, controller, observer, 11.0, DT);

    private final List<Double> characterizationData = new LinkedList<>();

    private final RobotContainer container = new RobotContainer();

    @Override
    public void robotInit() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 11.0;

        // Slot 0 is motion magic
        config.slot0.kP = STEER_MM_P;
        config.slot0.kI = STEER_MM_I;
        config.slot0.kD = STEER_MM_D;
        config.slot0.kF = STEER_MM_F;
        config.motionCruiseVelocity = MAX_PROFILE_VELOCITY / SENSOR_VELOCITY_COEFFICIENT;
        config.motionAcceleration = MAX_PROFILE_ACCELERATION / SENSOR_VELOCITY_COEFFICIENT;

//        ssMotor.configAllSettings(config);
//        pidMotor.configAllSettings(config);

//        ssMotor.enableVoltageCompensation(true);
//        pidMotor.enableVoltageCompensation(true);

//        ssMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
//        pidMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());

        ShuffleboardTab tab = Shuffleboard.getTab("Main");
//        ShuffleboardLayout stateSpaceLayout = tab.getLayout("State Space", BuiltInLayouts.kList);
//        stateSpaceLayout.addNumber("Estimated Position", () -> Math.toDegrees(loop.getXHat(0)));
//        stateSpaceLayout.addNumber("Estimated Velocity", () -> Math.toDegrees(loop.getXHat(1)));
//        stateSpaceLayout.addNumber("Actual Position", () -> Math.toDegrees(ssMotor.getSelectedSensorPosition() * SENSOR_POSITION_COEFFICIENT));
//        stateSpaceLayout.addNumber("Actual Velocity", () -> Math.toDegrees(ssMotor.getSelectedSensorVelocity() * SENSOR_VELOCITY_COEFFICIENT));
//        stateSpaceLayout.addNumber("Output", () -> loop.getU(0));


        ShuffleboardLayout pidLayout = tab.getLayout("PID", BuiltInLayouts.kList);
//        pidLayout.addNumber("Position", () -> Math.toDegrees(pidMotor.getSelectedSensorPosition() * SENSOR_POSITION_COEFFICIENT));
//        pidLayout.addNumber("Velocity", () -> Math.toDegrees(pidMotor.getSelectedSensorVelocity() * SENSOR_VELOCITY_COEFFICIENT));

//        loop.reset(VecBuilder.fill(
//                ssMotor.getSelectedSensorPosition() * SENSOR_POSITION_COEFFICIENT,
//                ssMotor.getSelectedSensorVelocity() * SENSOR_VELOCITY_COEFFICIENT
//        ));

//        addPeriodic(() -> {
//            if (profile != null) {
//                var state = profile.calculate(Timer.getFPGATimestamp() - profileStartTime);
//                loop.setNextR(state.position, state.velocity);
//                loop.predict(DT);
//            } else {
//                loop.getObserver().predict(VecBuilder.fill(0.0), DT);
//            }
//
//            ssMotor.set(TalonFXControlMode.PercentOutput, loop.getU(0) / 12.0);
//        }, DT);
    }

    @Nullable
    private MotionProfile profile = null;
    private double profileStartTime = 0.0;

    @Override
    public void robotPeriodic() {
//        loop.correct(VecBuilder.fill(
//                ssMotor.getSelectedSensorPosition() * SENSOR_POSITION_COEFFICIENT,
//                ssMotor.getSelectedSensorVelocity() * SENSOR_VELOCITY_COEFFICIENT
//        ));
    }

    @Override
    public void teleopInit() {
        // Reset encoders
//        ssMotor.setSelectedSensorPosition(0.0);
//        pidMotor.setSelectedSensorPosition(0.0);
//
//        loop.reset(VecBuilder.fill(
//                ssMotor.getSelectedSensorPosition() * SENSOR_POSITION_COEFFICIENT,
//                ssMotor.getSelectedSensorVelocity() * SENSOR_VELOCITY_COEFFICIENT
//        ));
        profile = new TrapezoidalMotionProfile(
                new MotionProfile.Goal(loop.getXHat(0), loop.getXHat(1)),
                new MotionProfile.Goal(Math.toRadians(90.0), 0.0),
                new MotionProfile.Constraints(MAX_PROFILE_VELOCITY, MAX_PROFILE_ACCELERATION)
        );
        profileStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void teleopPeriodic() {
//        pidMotor.set(TalonFXControlMode.MotionMagic, Math.toRadians(90.0) / SENSOR_POSITION_COEFFICIENT);

        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        profile = null;

        StringBuilder builder = new StringBuilder();
        for (double v : characterizationData) {
            if (builder.length() != 0)
                builder.append(',');
            builder.append(v);
        }

        SmartDashboard.putString("SysIdTelemetry", builder.toString());
        characterizationData.clear();
    }

    @Override
    public void autonomousInit() {
        SmartDashboard.putString("SysIdTelemetry", "");
        // Reset encoders
//        ssMotor.setSelectedSensorPosition(0.0);
//        pidMotor.setSelectedSensorPosition(0.0);
    }

    @Override
    public void autonomousPeriodic() {
        double speed = SmartDashboard.getNumber("SysIdAutoSpeed", 0.0);
        double voltage = RobotController.getInputVoltage();

//        pidMotor.set(TalonFXControlMode.PercentOutput, speed);

//        characterizationData.add(Timer.getFPGATimestamp());
//        characterizationData.add(speed * voltage);
//        characterizationData.add(pidMotor.getSelectedSensorPosition() * SENSOR_POSITION_COEFFICIENT / 2.0 * Math.PI);
//        characterizationData.add(pidMotor.getSelectedSensorVelocity() * SENSOR_VELOCITY_COEFFICIENT / 2.0 * Math.PI);
    }
}
