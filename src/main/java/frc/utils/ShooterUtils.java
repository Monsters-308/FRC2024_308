package frc.utils;

import edu.wpi.first.math.geometry.Translation2d;
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

    public static double shooterAngleToFacePoint(Translation2d pivotPosition, Translation2d goalPosition){
        // TODO: implement this function
        double pivotAngle = pivotAngleToFacePoint(goalPosition);
        double pivotDistance = pivotDistanceToPoint(goalPosition);

        return 0;
    }
    
}
