package frc.robot;
import com.revrobotics.spark.SparkMax; //revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkLowLevel; //com.revrobotics.CANSparkLowLevel;

public class Dorito {
    SparkMax motor1;
    SparkMax motor2;
    
    public Dorito() {
        //this.motor1 = new CANSparkMax(Constants.dorito1ID, CANSparkLowLevel.MotorType.kBrushless);
        //this.motor2 = new CANSparkMax(Constants.dorito2ID, CANSparkLowLevel.MotorType.kBrushless);
        //motor1.setInverted(true);
        //motor2.setInverted(true);
    }

    public void runForwards() {
       // motor1.set(1);
        motor2.set(.25);
    }

    public void lightShot() {
        motor1.set(.3);
        motor2.set(.3);
    }

    public void runBackwards() {
        motor1.set(-.4);
        motor2.set(-.4);
    }

    public void stopMotors() {
        motor1.stopMotor();
        motor2.stopMotor();
    }
}
