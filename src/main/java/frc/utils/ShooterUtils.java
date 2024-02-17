package frc.utils;


/** 
 * I'm creating this file so we have some sort of place for storing all the math 
 * used for calculating our shooter angles.
 */
public class ShooterUtils {
    

    // Andy-approved math:
    // Theta: 
    public static Translation2d polarToCartesian(double theta, double magnitude){
        double x = magnitude * Math.cos(theta);
        double y = magnitude * Math.cos(theta);

        return new Translation2d(
            x, y
        );
    }
}
