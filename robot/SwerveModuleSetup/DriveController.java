package frc.robot.SwerveModuleSetup;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
