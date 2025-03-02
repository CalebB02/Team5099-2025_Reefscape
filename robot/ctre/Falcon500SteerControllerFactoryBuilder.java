package frc.robot.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import frc.robot.SwerveModuleSetup.AbsoluteEncoder;
import frc.robot.SwerveModuleSetup.AbsoluteEncoderFactory;
import frc.robot.SwerveModuleSetup.ModuleConfiguration;
import frc.robot.SwerveModuleSetup.SteerController;
import frc.robot.SwerveModuleSetup.SteerControllerFactory;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import static frc.robot.ctre.CtreUtils.checkCtreError;

public final class Falcon500SteerControllerFactoryBuilder {
    private static final double TICKS_PER_ROTATION = 2048.0;

    private double proportionalConstant = Double.NaN;
    private double integralConstant = Double.NaN;
    private double derivativeConstant = Double.NaN;

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
        public void addDashboardEntries(ShuffleboardContainer container, ControllerImplementation controller) {
            SteerControllerFactory.super.addDashboardEntries(container, controller);
            container.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(controller.absoluteEncoder.getAbsoluteAngle()));
        }

        @Override
        public ControllerImplementation create(Falcon500SteerConfiguration<T> steerConfiguration, ModuleConfiguration moduleConfiguration) {
            AbsoluteEncoder absoluteEncoder = encoderFactory.create(steerConfiguration.getEncoderConfiguration());
            final double sensorPositionCoefficient = 2.0 * Math.PI / TICKS_PER_ROTATION * moduleConfiguration.getSteerReduction();

            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            var slot0Configs = motorConfiguration.Slot0;
            if (hasPidConstants()) {
                slot0Configs.kP = proportionalConstant;
                slot0Configs.kI = integralConstant;
                slot0Configs.kD = derivativeConstant;
            }

            TalonFX motor = new TalonFX(steerConfiguration.getMotorPort());
            motor.getConfigurator().apply(motorConfiguration);
            motor.setNeutralMode(NeutralModeValue.Brake);
            motor.setInverted(moduleConfiguration.isSteerInverted());
            return new ControllerImplementation(motor, sensorPositionCoefficient, absoluteEncoder);
        }
    }

    private static class ControllerImplementation implements SteerController {
        private final TalonFX motor;
        private final double motorEncoderPositionCoefficient;
        private final AbsoluteEncoder absoluteEncoder;
        private double referenceAngleRadians = 0.0;

        private ControllerImplementation(TalonFX motor, double motorEncoderPositionCoefficient, AbsoluteEncoder absoluteEncoder) {
            this.motor = motor;
            this.motorEncoderPositionCoefficient = motorEncoderPositionCoefficient;
            this.absoluteEncoder = absoluteEncoder;
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            motor.setControl(new PositionVoltage(referenceAngleRadians / motorEncoderPositionCoefficient));
            this.referenceAngleRadians = referenceAngleRadians;
        }

        @Override
        public double getStateAngle() {
            double motorAngleRadians = motor.getPosition().getValueAsDouble() * motorEncoderPositionCoefficient;
            motorAngleRadians %= 2.0 * Math.PI;
            if (motorAngleRadians < 0.0) {
                motorAngleRadians += 2.0 * Math.PI;
            }
            return motorAngleRadians;
        }
    }
}
