package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Limelight {
    public static double[] calcDistance(String target) {
        double goalHeight = 0.0;
        if (target.equals("amp")) goalHeight = Constants.ampHeight;
        if (target.equals("speaker")) goalHeight = Constants.speakerHeight;
        if (target.equals("stage")) goalHeight = Constants.stageHeight;
        if (target.equals("test")) goalHeight = Constants.testHeight;

        double targetOffsetAngle_Vertical = LimelightHelpers.getTY(Constants.limelightName);
        System.out.println("TY: "+targetOffsetAngle_Vertical);

        double angleToGoalRadians = Math.toRadians(Constants.limelightMountAngleDegrees + targetOffsetAngle_Vertical);

        //calculate distance
        double dy = (goalHeight - Constants.limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        // double goalDistance = 0.0;
        // if (target.equals("amp")) goalDistance = Constants.ampOffsetFromAprilTag;
        // if (target.equals("speaker")) goalDistance = Constants.speakerOffsetFromAprilTag;
        // if (target.equals("stage")) goalDistance = Constants.stageOffsetFromAprilTag;

        // double targetOffsetAngle_Horizontal = LimelightHelpers.getTX(Constants.limelightName);
        // double angleToGoalRadiansHorizontal = Math.toRadians(Constants.limelightHorizontalMountAngleDegrees + targetOffsetAngle_Horizontal);
        // double dx = (dy - Constants.limelightLensHorziontalOffset) / Math.tan(angleToGoalRadiansHorizontal);
        double horizontalAngle = Math.toRadians(LimelightHelpers.getTX(Constants.limelightName));
        System.out.println("TX: "+ horizontalAngle);
        return new double[]{dy*Math.tan(horizontalAngle), dy};
    }

    // public static double alignRobot() {
    //     double KpDistance = -0.1;  // Proportional control constant for distance
    //     double[] current_distance = calcDistance("amp");
    //     double desired_distance = 20;
        
    //     double distance_error = desired_distance - current_distance[1];
    //     double driving_adjust = KpDistance * distance_error;
            
    //     return driving_adjust;
    // }
}
