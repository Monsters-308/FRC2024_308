// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kMagnitudeSlewRate = 1.8 * kMaxSpeedMetersPerSecond; // meters per second^2
    public static final double kRotationalSlewRate = 2.0 * kMaxAngularSpeed;        // radians per second^2

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21); // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21); // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 7;
  }

  // This is specifically for constants related to the individual swerve modules and not to the drive subsystem itself.
  public static final class ModuleConstants {

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // The L1 MK4 and MK4i modules have a gear ratio of 8.14:1 on the drive wheels.
    public static final double kDrivingMotorReduction = 8.14;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    //TODO: We need to figure out how feedforwards work and how to use them.
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps; 
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    // Inversion of drive motors
    // This will vary depending on how your wheels are oriented when you zero them.
    public static final boolean kLeftFrontInverted = true;
    public static final boolean kLeftRearInverted = true;
    public static final boolean kRightFrontInverted = true;
    public static final boolean kRightRearInverted = false;

    // Inversion of turning motors
    // Unless oriented differently, all of your turning motors should spin in the same direction.
    public static final boolean kTurningMotorsInverted = true;

    // Inversion of turning ENCODERS (not motors).
    // Unless oriented differently, all of your turning encoders should spin in the same direction.
    public static final boolean kTurningEncoderInverted = false;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    //TODO: probably not super important, but we should look into how to properly calculate current limits.
    public static final int kDrivingMotorCurrentLimit = 35; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class HeadingConstants {
    // The gyro should be CCW positive
    public static final boolean kGyroReversed = true;

    // This is used for making the robot face a certain direction
    public static final double kHeadingP = 0.05;
    public static final double kHeadingI = 0;
    public static final double kHeadingD = 0.001;
    public static final double kHeadingMaxOutput = 0.5; // Percent
    public static final double kHeadingTolerance = 1; // Degrees

    // TODO: remove the old auton constants and put these in there
    public static final double kTranslationP = 5;
    public static final double kTranslationI = 0;
    public static final double kTranslationD = 0;
    public static final double kTranslationMaxOutput = 0.5; // Percent
    public static final double kTranslationTolerance = Units.inchesToMeters(3); // Meters
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class FieldConstants{
    /** X axis: long side */
    public static final double kFieldWidthMeters = 16.52;
    /** Y axis: short side */
    public static final double KFieldHeightMeters = 8.2;
    
    public static final double kSpeakerX = 0; // Change
    public static final double kSpeakerY = 0; // Change
  }

  public static final class AutoConstants {
    // TODO: these are old example constants. Figure out which to keep and which to get rid of.
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    //How many degrees is your limelight rotated from perfectly vertical
    public static final double kLimelightMountAngle = 0; 

    //Limelight lens height from floor in inches
    public static final double kLimelightLensHeight = 20;

    //Height of reflective tape poles in inches
    public static final double kSpeakerHeightFromLimelightHeightInches = 71.25;
    public static final double kBottomReflectiveTapeHeight = 24;
    public static final double kLimelightDistanceFromCenterInches = 4.5;
    

    public static final double kTopPoleDesiredDistance = 24;
    public static final double kDistanceTolerance = 2;
    public static final double kMaxForwardSpeed = 0.7;
    public static final double kForwardSpeedPConstant = 0.1;

    public static final double kRotationSpeed = 0.2;
    public static final double kRotationTolerance = 2;

    //Pipeline constants
    public static final int kAprilTagPipeline = 0;
    public static final int kReflectiveTapePipeline = 3;
    public static final int kGamePiecePipeline = 2;

    /* NOTE: the limelight starts with pipeline 0 by default, so we need to make sure we make that pipeline something 
     * that doesn't use the green lights so we don't blind everybody.
     */
    public static final int kDefaultPipeline = kAprilTagPipeline;
  }

  public static final class ArmConstants {

  }

  public static final class HangingConstants {
    public static final int kRightArmCanID = 17;
    public static final int kRightArmUpperLimit = 8;
    public static final int kRightArmLowerLimit = 9;

    public static final int kLeftArmCanID = 18;
    public static final int kLeftArmUpperLimit = 8;
    public static final int kLeftArmLowerLimit = 9;

    // Forwards should be extension, Reverse should be retraction
    public static final boolean kHangingMotorInverted = false;
    public static final int kHangingMotorCurrentLimit = 35;

  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorCanID = 4; // Change

    // Positive intakes the piece, negative retracts the piece
    public static final boolean kIntakeMotorInverted = false; // Change
    public static final int kIntakeMotorCurrentLimit = 35; // Change
  }

  public static final class IndexConstants {
    public static final int kIndexMotorCanID = 4; // Change

    // Positive should bring the game piece to the shooter
    public static final boolean kIndexMotorInverted = false; // Change
    public static final int kIndexMotorCurrentLimit = 35; // Change
  }

  public static final class ShooterConstants {

    public static final double kShooterEncoderVelocityFactor = (2 * Math.PI); // radians //TODO: this is assuming that the native unit for the encoder is revolutions. Double check
        
    public static final int kTopShooterMotorCanID = 16; // Change
    public static final int kBottomShooterMotorCanID = 17; // Change
    
    public static final IdleMode kTopShooterMotor = IdleMode.kCoast;
    public static final IdleMode kBottomShooterMotor = IdleMode.kCoast;

    public static final int kShooterMotorCurrentLimit = 35; // amps

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // The L1 MK4 and MK4i modules have a gear ratio of 8.14:1 on the drive wheels.
    public static final double kDrivingMotorReduction = 1;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kShooterP = 0.04;
    public static final double kShooterI = 0;
    public static final double kShooterD = 0;
    //TODO: We need to figure out how feedforwards work and how to use them.
    public static final double kShooterFF = 1 / kDriveWheelFreeSpeedRps; 
    public static final double kShooterMinOutput = -1;
    public static final double kShooterMaxOutput = 1;

    public static final int kDigitalSensorPin = 10;
    
    public static final boolean kSensorInverted = true; 

    //Conveyer Belt constants 
    public static final double belt_speed = .01; 
  }

  public static final class ShooterPivotConstants {
    public static final int kShooterPivotMotorCanID = 10; // Change
    public static final boolean kTurningMotorInverted = false; // Change
    public static final boolean kTurningMotorEncoderInverted = false; // Change
    
    public static final double kShooterEncoderVelocityFactor = (2 * Math.PI); // radians //TODO: this is assuming that the native unit for the encoder is revolutions. Double check
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kShooterEncoderVelocityFactor; // radians
    
    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;
    
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
    public static final int kDrivingMotorCurrentLimit = 35; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    public static final boolean kMotorInverted = false;
  }
}