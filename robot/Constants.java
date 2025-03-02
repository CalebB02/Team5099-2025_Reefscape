package frc.robot;

public class Constants {
  public static final double sens = 2; //in volts
  public static final double centerDistance = .300;

  public static final double kP = 0.5;
  public static final double kI = 0; // want motor to stop when at correct angle, so keep at 0
  public static final double kD = 0; //multiplier based on rate of change of angle difference

  //CAN IDs for Swerve Drive Modules
  public static final int frontLeftDriveID = 3; //has noise
  public static final int backLeftDriveID = 2; //less noisy than first
  public static final int frontRightDriveID = 5; //a little better than previous
  public static final int backRightDriveID = 4; 

  public static final int frontLeftAngleID = 7;
  public static final int backLeftAngleID = 1;
  public static final int frontRightAngleID = 6;
  public static final int backRightAngleID = 8;

  public static final int intake1ID = 10;
  public static final int intake2ID = 11;
  public static final int intake3ID = 12;

  public static final int wristID = 13;

  public static final int aumMotorID = 14;

  public static final int shooter1ID = 15;
  public static final int shooter2ID = 16;

  public static final int frontLeftCANID = 2;
  public static final int backLeftCANID = 3;
  public static final int frontRightCANID = 1;
  public static final int backRightCANID = 4;

  //Offsets for Swerve Drive Modules
  public static final double frontLeftOffset = 5.7248;
  public static final double backLeftOffset = 2.2562;
  public static final double frontRightOffset = 2.6456;
  public static final double backRightOffset = 5.6159;

  //Limelight constants
  public static final String limelightName = "limelight-iofmike";

  // how many degrees back is your limelight rotated from perfectly vertical?
  public static final double limelightMountAngleDegrees = 0; 
  
  // distance from the center of the Limelight lens to the floor
  public static final double limelightLensHeightInches = 20.5; //8.5; 
  
  // distance from the target to the floor
  public static final double testHeight = 33;
  public static final double ampHeight = 48.125; 
  public static final double speakerHeight = 48.125;
  public static final double stageHeight = 47.5;

  public static final double speakerDistance = 51.5;

  // Horizontal version
  public static final double limelightHorizontalMountAngleDegrees = 0.0;
  public static final double limelightLensHorziontalOffset = 0.0;

  public static final double ampOffsetFromAprilTag = 0;
  public static final double speakerOffsetFromAprilTag = 0;
  public static final double stageOffsetFromAprilTag = 0;
}
