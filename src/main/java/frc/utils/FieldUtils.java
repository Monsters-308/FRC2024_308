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
}
