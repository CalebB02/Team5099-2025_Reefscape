package frc.robot;

//this is a line of code! i am a professional coder.
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.SwerveModuleSetup.Mk4iSwerveModuleHelper;
import frc.robot.SwerveModuleSetup.SwerveModule;
import frc.robot.SwerveModuleSetup.Mk4iSwerveModuleHelper.GearRatio;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.IOException;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

public class Robot extends TimedRobot {

  private static double previousVoltage = 0.0; 
  private static double previousAngle = 0.0;
  private static double offset = 0.0;
  private static int moduleNum = 0;
  private static boolean isTurning = false;

  private static SwerveModule frontLeft;
  private static SwerveModule backLeft;
  private static SwerveModule frontRight;
  private static SwerveModule backRight;  
  private static SwerveModule[] modules = new SwerveModule[4];

  private static SwerveDriveMovement driveController;
  private static ADXRS450_Gyro gyro;
  private static Dorito dorito;

  private static boolean isHolding = false;

  XboxController controller = new XboxController(0);


  DoubleLogEntry voltLog;
  DoubleLogEntry angleLog;

  Timer timer;

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture(); //starts camera with name: USB Camera 0
    gyro = new ADXRS450_Gyro();
    //dorito = new Dorito();
    //climby = new Climber();
     frontLeft = Mk4iSwerveModuleHelper.createNeo(GearRatio.L2, Constants.frontLeftDriveID, 
       Constants.frontLeftAngleID, Constants.frontLeftCANID, 0.0);
     modules[0] = frontLeft;

     backLeft = Mk4iSwerveModuleHelper.createNeo(GearRatio.L2, Constants.backLeftDriveID, 
       Constants.backLeftAngleID, Constants.backLeftCANID, 0.0);
     modules[1] = backLeft;

     frontRight = Mk4iSwerveModuleHelper.createNeo(GearRatio.L2, Constants.frontRightDriveID, 
       Constants.frontRightAngleID, Constants.frontRightCANID, 0.0);
     modules[2] = frontRight;

     backRight = Mk4iSwerveModuleHelper.createNeo(GearRatio.L2, Constants.backRightDriveID, 
       Constants.backRightAngleID,Constants.backRightCANID, 0.0);
     modules[3] = backRight;

    driveController = new SwerveDriveMovement(frontLeft, backLeft, frontRight,
     backRight);

    //SmartDashboard.putData("Gyro", gyro);
    
    // DataLogManager.start();
    // DataLog log = DataLogManager.getLog();
    // DriverStation.startDataLog(log);
    // voltLog = new DoubleLogEntry(log, "/drivedata/driveVoltage");
    // angleLog = new DoubleLogEntry(log, "/drivedata/driveAngle");
    // try {
    //   CreateFile.createLog();
    // } catch (IOException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    SmartDashboard.putData("Gyro", gyro);

    //Limelight port forwarding
    for (int port = 5800; port <= 5807; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }
  }

  @Override
  public void testInit() {}

  @Override 
  public void testPeriodic() { //Use test mode in Driver Station to manually align the wheels
    if (controller.getAButton()) {
      offset+=.1;
    }
    else if (controller.getBButton()) {
      offset-=.1;
    }
    else if (controller.getYButton() && moduleNum < 3) {
      moduleNum++;
    }
    else if (controller.getXButton() && moduleNum > 0) {
      moduleNum--;
    }


    switch (moduleNum) {
      case 0: System.out.print("frontLeft"); break;
      case 1: System.out.print("backLeft"); break;
      case 2: System.out.print("frontRight"); break;
      case 3: System.out.print("backRight"); break;
      default: System.out.print("Out of range");
    }

    modules[moduleNum].set(0.0, offset); //comment this out to manually align

    System.out.println("\nfrontLeft: "+frontLeft.getSteerAngle());
    System.out.println("backLeft: "+backLeft.getSteerAngle());
    System.out.println("frontRight: "+frontRight.getSteerAngle());
    System.out.println("backRight: "+backRight.getSteerAngle());
    System.out.println("\nCurrentModule: ");
  }

  @Override 
  public void autonomousInit() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    // Autonomous.autoSpeaker(driveController, shooter, timer);
  }

  @Override
  public void teleopPeriodic() {

    double x = controller.getLeftX();
    double y = controller.getLeftY();
    boolean leftStickPressed = x > .1 || x < -.1 || y > .1 || y < -.1;

    double driveVoltage = 0.0;
    double steerAngle = 0.0;

    // if (leftStickPressed) {
    //   //System.out.println("regular voltage: "+driveVoltage);
    //   //System.out.println("converted voltage: "+speedConvert(driveVoltage, steerAngle));
    // }

    // Aligns robot with target
    // if (controller.getRightStickButton()) {
    //   double[] distances = Limelight.calcDistance("speaker");
    //   System.out.println("dx: "+distances[0]+" dy: "+distances[1]);
    //   double kpDistance = .05;
    //   double dyError = Math.abs((distances[1]-Constants.speakerDistance) * kpDistance)+.2;
    //   double dxError = Math.abs((distances[0]) * kpDistance)+.2;
    //   double alignAngle= Math.atan(Math.abs(dyError)/dxError);
    //   double alignSpeed= Math.sqrt(Math.pow(dyError,2)+Math.pow(dxError,2))*kpDistance;
    //   if (dyError < 2) dyError = 0;
    //   if (dxError < 2) dxError = 0;

    //   alignSpeed = Math.min(alignSpeed, Constants.sens);

    //   if (distances[1]-Constants.speakerDistance < 0) alignSpeed*=-1;
    //   driveController.translation(alignSpeed, alignAngle);
    // }
    // else {
    if (leftStickPressed) {
      steerAngle = Math.atan(x/y);
      if (y > 0) {
        steerAngle+= Math.PI;
        //driveVoltage*=-1;
      }
      //steerAngle = Math.atan2(x, y);

    if (Math.abs(steerAngle-previousAngle) < .1) {
      steerAngle = previousAngle;
    }
      driveVoltage = Math.min(Constants.sens*Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)), Constants.sens);
      if (Math.abs(driveVoltage-previousVoltage) > 1) {
        driveVoltage = driveVoltage>previousVoltage?previousVoltage+1:previousVoltage-1;
      }
      //driveController.translation(driveVoltage, steerAngle);
    }

    if (controller.getRightBumper() || controller.getLeftBumper()) {
      driveController.rotation(driveVoltage, steerAngle, controller.getRightBumper(), leftStickPressed);
      isTurning = true;
    } 
    else if (isTurning) {
      driveController.stopMotors();
      isTurning = false;
    }
      
    if (!leftStickPressed && steerAngle != previousAngle) {
      driveController.translation(0, previousAngle);
    }
    else if (steerAngle != previousAngle || driveVoltage != previousVoltage && !isTurning) {
      driveController.translation(driveVoltage, steerAngle);
      // voltLog.append(driveVoltage, (long)DriverStation.getMatchTime());
      // angleLog.append(steerAngle, (long)DriverStation.getMatchTime());
    }

    if (controller.getBButtonPressed()) {
      driveController.translation(0,0);
    }

    // System.out.println("X: " + x);
    // System.out.println("y: " + y);
    // System.out.println("Steerangle" + steerAngle);
    System.out.println("driveVoltage: " + driveVoltage);

    previousVoltage = driveVoltage;
    previousAngle = steerAngle;

    // try {
    //   CreateFile.addValues(DriverStation.getMatchTime(), driveVoltage, steerAngle);
    // } catch (IOException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    
    } 
}
