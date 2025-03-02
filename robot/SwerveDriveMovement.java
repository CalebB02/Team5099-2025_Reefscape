package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveModuleSetup.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

//import edu.wpi.first.math.geometry.Rotation2d; might need this for the module angles
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDriveMovement {

    private final double CENTERDISTANCE = Constants.centerDistance; //Length/width to center of robot from swerve modules in meters

    private SwerveModule frontLeft;
    private SwerveModule backLeft;
    private SwerveModule frontRight;
    private SwerveModule backRight;    

    private final Translation2d frontLeftDistance = new Translation2d(CENTERDISTANCE, CENTERDISTANCE);
    private final Translation2d backLeftDistance = new Translation2d(-CENTERDISTANCE, CENTERDISTANCE);
    private final Translation2d frontRightDistance = new Translation2d(CENTERDISTANCE, -CENTERDISTANCE);
    private final Translation2d backRightDistance = new Translation2d(-CENTERDISTANCE, -CENTERDISTANCE);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftDistance, backLeftDistance, frontRightDistance, backRightDistance);

    public SwerveDriveMovement(SwerveModule frontLeft, SwerveModule backLeft, SwerveModule frontRight,
     SwerveModule backRight) {
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
    }

    public void rotation(double speed, double angle, boolean isRight, boolean isMoving) {
        //Checking if robot is moving to do normal rotation or rotation while moving
        if (isMoving)
            rotationWhileMoving(speed, angle, isRight);
        else
            rotationInPlace(isRight);
    }

    public void rotationWhileMoving(double speed, double angle, boolean isRight) {
        double xSpeed = speed*Math.cos(angle);
        double ySpeed = speed*Math.sin(angle);
        double angleSpeed = isRight? -3.0:3.0;

        //Calculating the speeds and angles
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed,ySpeed,angleSpeed);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        double frontLeftSpeed = moduleStates[0].speedMetersPerSecond;
        double frontLeftAngle = moduleStates[0].angle.getRadians();

        double backLeftSpeed = moduleStates[1].speedMetersPerSecond;
        double backLeftAngle = moduleStates[1].angle.getRadians();

        double frontRightSpeed = moduleStates[2].speedMetersPerSecond;
        double frontRightAngle = moduleStates[2].angle.getRadians();

        double backRightSpeed = moduleStates[3].speedMetersPerSecond;
        double backRightAngle = moduleStates[3].angle.getRadians();

        frontLeft.set(frontLeftSpeed, frontLeftAngle-Constants.frontLeftOffset);
        backLeft.set(backLeftSpeed, backLeftAngle-Constants.backLeftOffset);
        frontRight.set(frontRightSpeed, frontRightAngle-Constants.frontRightOffset);
        backRight.set(backRightSpeed, backRightAngle-Constants.backRightOffset);
    }

    public void rotationInPlace(boolean isRight) {
        double speed = 2.0;
        if (isRight)
            speed = -speed;
        
        frontLeft.set(speed,3*Math.PI/4-Constants.frontLeftOffset);
        backLeft.set(speed,Math.PI/4-Constants.backLeftOffset);
        frontRight.set(speed,5*Math.PI/4-Constants.frontRightOffset);
        backRight.set(-speed,3*Math.PI/4-Constants.backRightOffset);
    }

    public void translation(double speed, double angle) {
        frontLeft.set(speed, angle-Constants.frontLeftOffset);
        backLeft.set(speed, angle-Constants.backLeftOffset);
        frontRight.set(speed, angle-Constants.frontRightOffset);
        backRight.set(speed, angle-Constants.backRightOffset);
    }

    public void getDriveVelocity() {
        System.out.println(frontLeft.getDriveVelocity());
    }

    public void stopMotors() {
        translation(0.0,0.0);
      }
    }