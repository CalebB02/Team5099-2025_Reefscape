package frc.robot.rev;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.SwerveModuleSetup.AbsoluteEncoder;
import frc.robot.SwerveModuleSetup.AbsoluteEncoderFactory;
import frc.robot.SwerveModuleSetup.ModuleConfiguration;
import frc.robot.SwerveModuleSetup.SteerController;
import frc.robot.SwerveModuleSetup.SteerControllerFactory;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import static frc.robot.rev.RevUtils.checkNeoError;

public final class NeoSteerControllerFactoryBuilder {
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

            SparkMax motor = new SparkMax(steerConfiguration.getMotorPort(), SparkLowLevel.MotorType.kBrushless);
            SparkMaxConfig steerConfig = new SparkMaxConfig();
            //checkNeoError(motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, 100), "Failed to set periodic status frame 0 rate");
            //checkNeoError(motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 20), "Failed to set periodic status frame 1 rate");
            //checkNeoError(motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 20), "Failed to set periodic status frame 2 rate");
            //checkNeoError(motor.setIdleMode(SparkMax.IdleMode.kBrake), "Failed to set NEO idle mode");
            motor.setInverted(!moduleConfiguration.isSteerInverted());
            if (hasVoltageCompensation()) {
                //checkNeoError(motor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");
                steerConfig.voltageCompensation(nominalVoltage);
            }
            if (hasCurrentLimit()) {
                //checkNeoError(motor.setSmartCurrentLimit((int) Math.round(currentLimit)), "Failed to set NEO current limits");
                steerConfig.smartCurrentLimit((int) Math.round(currentLimit));
            }

            RelativeEncoder integratedEncoder = motor.getEncoder();
            //checkNeoError(integratedEncoder.setPositionConversionFactor(2.0 * Math.PI * moduleConfiguration.getSteerReduction()), "Failed to set NEO encoder conversion factor");
            //checkNeoError(integratedEncoder.setVelocityConversionFactor(2.0 * Math.PI * moduleConfiguration.getSteerReduction() / 60.0), "Failed to set NEO encoder conversion factor");
            steerConfig.encoder
                .positionConversionFactor(2.0 * Math.PI * moduleConfiguration.getSteerReduction()) // meters
                .velocityConversionFactor(2.0 * Math.PI * moduleConfiguration.getSteerReduction() / 60.0);
            checkNeoError(integratedEncoder.setPosition(absoluteEncoder.getAbsoluteAngle()), "Failed to set NEO encoder position");

            SparkClosedLoopController controller = motor.getClosedLoopController();
            // if (hasPidConstants()) {
            //     checkNeoError(controller.setP(pidProportional), "Failed to set NEO PID proportional constant");
            //     checkNeoError(controller.setI(pidIntegral), "Failed to set NEO PID integral constant");
            //     checkNeoError(controller.setD(pidDerivative), "Failed to set NEO PID derivative constant");
            // }
            //checkNeoError(controller.setFeedbackDevice(integratedEncoder), "Failed to set NEO PID feedback device");

            return new ControllerImplementation(motor, absoluteEncoder);
        }
    }

    public static class ControllerImplementation implements SteerController {
        private static final int ENCODER_RESET_ITERATIONS = 500;
        private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

        @SuppressWarnings({"FieldCanBeLocal", "unused"})
        private final SparkMax motor;
        private final SparkClosedLoopController controller;
        private final RelativeEncoder motorEncoder;
        private final AbsoluteEncoder absoluteEncoder;

        private double referenceAngleRadians = 0;

        private double resetIteration = 0;

        public ControllerImplementation(SparkMax motor, AbsoluteEncoder absoluteEncoder) {
            this.motor = motor;
            this.controller = motor.getClosedLoopController();
            this.motorEncoder = motor.getEncoder();
            this.absoluteEncoder = absoluteEncoder;
            //motor.setClosedLoopRampRate(.25); //added this
            //motor.setSmartCurrentLimit(5); //added this
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            double currentAngleRadians = motorEncoder.getPosition();

            // Reset the NEO's encoder periodically when the module is not rotating.
            // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
            // end up getting a good reading. If we reset periodically this won't matter anymore.
            if (motorEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
                if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                    resetIteration = 0;
                    double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
                    motorEncoder.setPosition(absoluteAngle);
                    currentAngleRadians = absoluteAngle;
                }
            } else {
                resetIteration = 0;
            }

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

            controller.setReference(adjustedReferenceAngleRadians, SparkMax.ControlType.kPosition);
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
