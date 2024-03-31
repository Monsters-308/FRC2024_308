package frc.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterPivotConstants;

/** 
 * I'm creating this file so we have some sort of place for storing all the math 
 * used for calculating our shooter angles.
 */
public class ShooterUtils {

    /**
     * Takes an angle and magnitude and converts it to cartesian.
     * @param angle The angle in radians.
     * @param magnitude The magnitude.
     * @return The x and y relative to (0, 0).
     */
    public static Translation2d polarToCartesian(double angle, double magnitude){
        double x = Math.cos(angle) * magnitude;
        double y = Math.sin(angle) * magnitude;

        return new Translation2d(x, y);
    }

    /**
     * Returns the position of the shooter, in inches, relative to the shooter pivot.
     * @param currentAngle The current angle of the relative encoder in radians.
     * @return How many inches up and right from the shooter pivot.
     */
    public static Translation2d getShooterPosition(double currentAngle){
        return polarToCartesian(currentAngle + ShooterConstants.kAngleFromPivotToShooter, ShooterConstants.kDistanceFromPivotToShooter);
    }

    /**
     * Returns the angle to make the pivot face a point in space.
     * Note to Gabe: I made this function private because you would never use it outside of this class.
     * @param point A 2D point in space (in inches). The Y should be relative to the ground, 
     * but the X should be the horizontal distance between the pivot and the point in space.
     * @return The angle in radians.
     */
    private static double pivotAngleToFacePoint(Translation2d point){
        return Math.atan( (point.getY() - ShooterPivotConstants.kPivotHeightInches) / point.getX());
    }

    /**
     * Returns the distance in inches between the pivot and a point in space.
     * Note to Gabe: I made this function private because you would never use it outside of this class.
     * @param point A 2D point in space (in inches). The Y should be relative to the ground, 
     * but the X should be the horizontal distance between the pivot and the point in space.
     * @return The distance in inches.
     */
    private static double pivotDistanceToPoint(Translation2d point){
        return Math.hypot(point.getY() - ShooterPivotConstants.kPivotHeightInches, point.getX());
    }

    /**
     * Returns the angle (in radians) needed to make shooter face a certain point in space.
     * @param robotPosition The position of the robot on the field in meters.
     * @param goalPosition The position of the target on the field in meters.
     * @param goalHeight The height of the target in inches.
     * @return The angle in radians.
     */
    public static double shooterAngleToFacePoint(Translation2d robotPosition, Translation2d goalPosition, double goalHeight){

        // The horizontal distance in inches between the center of the robot and the target
        double robotDistance = Units.metersToInches(OdometryUtils.getDistancePosToPos(robotPosition, goalPosition)); // (Birds-eye view)

        SmartDashboard.putNumber("Auto Aim distance", robotDistance); // DEBUG
        
        // turns into distacne between pivot and target
        // Y axis now represents height (horizontal view)
        // returning the X and Y distacnes as a translation2d object
        Translation2d goalRelativeToPivot = new Translation2d(
            robotDistance-ShooterPivotConstants.kPivotCenterOffsetInches, goalHeight
        );

        // Calculates angle between pivot and goal (horizontal view)
        double pivotAngle = pivotAngleToFacePoint(goalRelativeToPivot);

        SmartDashboard.putNumber("Shooter Angle From Pivot", Math.toDegrees(pivotAngle)); // DEBUG

        // Calculate hypotenuse between pivot and goal (horizontal view)
        double pivotDistance = pivotDistanceToPoint(goalRelativeToPivot);

        // Epic math to adjust angle (horizontal view)
        double adjustmentAngle = Math.asin(ShooterConstants.kShooterVerticalOffset / pivotDistance);

        SmartDashboard.putNumber("Adjusted Shooter Angle", Math.toDegrees(pivotAngle + adjustmentAngle)); // DEBUG

        return pivotAngle + adjustmentAngle;
    }

    /**
     * Uses an equation to make the shooter aim at the speaker because we've given up on using geometry.
     * @param robotPosition The position of the robot on the field in meters.
     * @param goalPosition The position of the speaker on the field in meters.
     * @return The angle in degrees.
     */
    public static double shooterAngleToFacePoint(Translation2d robotPosition, Translation2d goalPosition){

        // The horizontal distance in inches between the center of the robot and the target
        double robotDistance = OdometryUtils.getDistancePosToPos(robotPosition, goalPosition); // (Birds-eye view)

        //SmartDashboard.putNumber("Auto Aim distance", robotDistance); // DEBUG

        // Equation for pivot angle
        double desiredAngle = 93.3 - 27.5*robotDistance + 3.16 * Math.pow(robotDistance, 2);

        SmartDashboard.putNumber("Desired Pivot Angle", desiredAngle); // DEBUG

        return desiredAngle;
    }
    
}
