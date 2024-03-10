package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ShooterPivotConstants;

public class ThroughBoreEncoder extends DutyCycleEncoder {

    private boolean IsEncoderInverted;
    private double angleOffset;
    /**
     * A duty cycle encoder specifically for the rev through bore encoder.
     * @param channel The channel on the roborio (where the through bore encoder is plugged into on the roborio 1-10).
     */
    public ThroughBoreEncoder(int channel, boolean inverted, double angleOffset) {
        super(channel);
        this.IsEncoderInverted = inverted;
        this.angleOffset = angleOffset; // Initialize the angle offset

        super.setDutyCycleRange(1 / ShooterPivotConstants.kEncoderPeriod, 
            (ShooterPivotConstants.kEncoderPeriod - 1) / ShooterPivotConstants.kEncoderPeriod);
    }
    
    public Rotation2d getDegrees() {
        double adjustedPosition = (IsEncoderInverted ? -1 : 1) * 
            getAbsolutePosition() * 360 + angleOffset;
        return Rotation2d.fromDegrees(SwerveUtils.angleConstrain(adjustedPosition));
    }

    public Rotation2d getRadians() {
        double adjustedPosition = (IsEncoderInverted ? -1 : 1) * 
          getAbsolutePosition() * (Math.PI)*2 + angleOffset;
        return Rotation2d.fromDegrees(SwerveUtils.angleConstrain(adjustedPosition));
    }
}