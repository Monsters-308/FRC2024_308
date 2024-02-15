package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class FieldUtils {
    /**
     * Returns the Alliance color.
     * @param nullable (Optional) Whether null should be returned if the alliance is invalid. 
     * If set to false, Blue is returned instead of null.
     * @return The Alliance color (or null/Blue if invalid).
     */
    public static Alliance getAlliance(boolean nullable){
        Alliance alliance = DriverStation.getAlliance().orElse(null);
        if (nullable || alliance != null){
            return alliance;
        }

        // Return Blue if alliance is null and nullable is false
        return Alliance.Blue;
    }

    /**
     * Returns the Alliance color.
     * @return The Alliance color (or Blue if invalid).
     */
    public static Alliance getAlliance(){
        return getAlliance(false);
    }

    /**
     * Returns whether or not the robot is on the red alliance.
     * @return true if on red alliance, false if on blue alliance or invalid.
     */
    public static boolean isRedAlliance(){
        return getAlliance() == Alliance.Red;
    }

    /**
     * Takes a pose object and returns it flipped on both the Y axis and X axis.
     * This is used for the Field widget.
     * @param pose The pose object relative to the blue side (in meters).
     * @return The new pose object, also relative to the blue side but shifted to the red side.
     */
    public static Pose2d redWidgetFlip(Pose2d pose){
        return new Pose2d(
            FieldConstants.kFieldWidthMeters-pose.getX(),
            FieldConstants.KFieldHeightMeters-pose.getY(),
            new Rotation2d(pose.getRotation().getRadians()-Math.PI)
        );
    }

    /**
     * Flips a translation object's Y value to line up on the red side.
     * @param position A translation object for the blue side.
     * @return The translation object mirrored for the red side.
     */
    public static Translation2d flipRed(Translation2d position){
        return new Translation2d(
            position.getX(),
            FieldConstants.KFieldHeightMeters-position.getY()
        );
    }

    /**
     * Flips a y coordinate so that it'll line up on the red side.
     * @param yPosition A number used to represent the y coordinate of a position.
     * @return The y value flipped.
     */
    public static double flipRedY(double yPosition){
        return FieldConstants.KFieldHeightMeters-yPosition;
    }

    /**
     * Mirrors a Rotation2d object so that left becomes right.
     * This is equivalent to multiplying the angle by -1.
     * @param angle A rotation2d object. 
     * @return The same object but flipped.
     */
    public static Rotation2d flipRedAngle(Rotation2d angle){
        return angle.times(-1);
    }

    /**
     * Mirrors an angle so that left becomes right.
     * This is equivalent to multiplying the angle by -1.
     * @param angleDegrees An angle in degrees.
     * @return The flipped angle.
     */
    public static double flipRedAngle(double angleDegrees){
        return -angleDegrees;
    }
}
