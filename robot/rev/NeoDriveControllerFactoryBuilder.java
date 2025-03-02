package frc.robot.rev;

import com.revrobotics.spark.SparkMax; //com.revrobotics.CANSparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkLowLevel; //com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

import frc.robot.SwerveModuleSetup.DriveController;
import frc.robot.SwerveModuleSetup.DriveControllerFactory;
import frc.robot.SwerveModuleSetup.ModuleConfiguration;

import static frc.robot.rev.RevUtils.checkNeoError;

public final class NeoDriveControllerFactoryBuilder {
    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public NeoDriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public NeoDriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }
    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer id, ModuleConfiguration moduleConfiguration) {
            SparkMax motor = new SparkMax(id, SparkLowLevel.MotorType.kBrushless);
            SparkMaxConfig driveConfig = new SparkMaxConfig();
            //SparkBaseConfig baseConfig = new SparkBaseConfig();
            
            //baseConfig.inverted(moduleConfiguration.isDriveInverted());
            motor.setInverted(moduleConfiguration.isDriveInverted());

            // Setup voltage compensation
            if (hasVoltageCompensation()) {
                //checkNeoError(motor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");
            }

            if (hasCurrentLimit()) {
                //checkNeoError(motor.setSmartCurrentLimit(40), "Failed to set current limit for NEO");
                driveConfig.smartCurrentLimit(40);
            }
            //checkNeoError(motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, 100), "Failed to set periodic status frame 0 rate");
            //checkNeoError(motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 20), "Failed to set periodic status frame 1 rate");
            //checkNeoError(motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 20), "Failed to set periodic status frame 2 rate");

            // Set neutral mode to brake
            //motor.setIdleMode(SparkMax.IdleMode.kBrake);
            driveConfig.idleMode(IdleMode.kBrake);
            

            // Setup encoder
            RelativeEncoder encoder = motor.getEncoder();
            double positionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction();
            //encoder.setPositionConversionFactor(positionConversionFactor);
            //encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);
            driveConfig.encoder
                    .positionConversionFactor(positionConversionFactor) // meters
                    .velocityConversionFactor(positionConversionFactor / 60.0);


            return new ControllerImplementation(motor, encoder);
        }
    }

    private static class ControllerImplementation implements DriveController {
        private final SparkMax motor;
        private final RelativeEncoder encoder;

        private ControllerImplementation(SparkMax motor, RelativeEncoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
            //motor.setClosedLoopRampRate(.1); //added this
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            //motor.set(voltage/12);
            motor.setVoltage(voltage);
        }

        @Override
        public double getStateVelocity() {
            return encoder.getVelocity();
        }
    }
}
