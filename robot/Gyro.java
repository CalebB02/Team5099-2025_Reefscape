package frc.robot;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Gyro {
    ADXRS450_Gyro gyro;
    
    public Gyro() {
        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
    }

    public void printGyro() {
        SmartDashboard.putData("Gyro", gyro);
        // double GyroAngle = gyro.getAngle();
        // System.out.println("Angle: " + GyroAngle);
        // double GyroRate = gyro.getRate();
        // System.out.println("Rate: " + GyroRate);
    }
    
}
