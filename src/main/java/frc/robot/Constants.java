// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    public static final double kMagnitudeSlewRate = 5 * kMaxSpeedMetersPerSecond; // meters per second^2
    public static final double kRotationalSlewRate = 5 * kMaxAngularSpeed;        // radians per second^2

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.75); // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.75); // Distance between front and back wheels on robot
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
    public static final int kFrontLeftDrivingCanId = 8;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 9;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 5;
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
    public static final boolean kLeftFrontInverted = false;
    public static final boolean kLeftRearInverted = true;
    public static final boolean kRightFrontInverted = false;
    public static final boolean kRightRearInverted = true;

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
    public static final int kTurningMotorCurrentLimit = 35; // amps
  }

  public static final class HeadingConstants {
    // The gyro should be CCW positive
    public static final boolean kGyroReversed = true;

    // This is used for making the robot face a certain direction
    // TODO: look make sure you test these
    public static final double kHeadingP = 0.05;
    public static final double kHeadingI = 0;
    public static final double kHeadingD = 0.001;
    public static final double kHeadingMaxOutput = 0.5; // Percent
    public static final double kHeadingTolerance = 1; // Degrees

    // TODO: remove the old auton constants and put these in there
    public static final double kTranslationP = 5;
    public static final double kTranslationI = 0;
    public static final double kTranslationD = 0;
    public static final double kTranslationMaxOutput = 0.5; // Percent // TODO: remember to set this to 1 after testing
    public static final double kTranslationTolerance = Units.inchesToMeters(3); // Meters
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
    
    public static final double kJoystickDeadband = 0.05;
    public static final double kTriggerDeadband = 0.5;
  }

  public static final class FieldConstants {
    /** X axis: long side */
    public static final double kFieldWidthMeters = 16.54175;
    /** Y axis: short side */
    public static final double kFieldHeightMeters = 8.2;
    
    // Position of the speaker on the field (in meters)
    public static final Translation2d kSpeakerPosition = new Translation2d(
      0, 5.54 
    );
    // Height to the bottom lip of speaker
    public static final double kSpeakerHeightInches = 78.129;

    public static final Pose2d kAmpScoringPosition = new Pose2d(
      1.83,
      7.67,
      Rotation2d.fromDegrees(90)
    );

    public static final Pose2d kSpeakerScoringPosition = new Pose2d(
      1.38,
      5.53,
      Rotation2d.fromDegrees(180)
    );
    
    // TODO: figure out trap constants
    public static final Pose2d kTrapPositionAmpSide = new Pose2d(
      0,
      0,
      Rotation2d.fromDegrees(0)
    );
    public static final Pose2d kTrapPositionSourceSide = new Pose2d(
      0,
      0,
      Rotation2d.fromDegrees(0)
    );
    public static final Pose2d kTrapPositionCenterStage = new Pose2d(
      0,
      0,
      Rotation2d.fromDegrees(0)
    );
    public static final double kTrapHeight = 0;

    public static final double kTrapCenterY = 4.1;
    public static final double kTrapCenterStageX = 5.63;
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

    public static final double kWheelSpeedAmp = 80;
    public static final double kAngleAmp = 60;
    public static final double kWheelSpeedSpeaker = 100;
    public static final double kAngleSpeaker = 50;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    // Pipeline constants
    public static final int kAprilTagPipeline = 0;
    // public static final int kReflectiveTapePipeline = 3;
    public static final int kGamePiecePipeline = 2;

    /* NOTE: the limelight starts with pipeline 0 by default, so we need to make sure we make that pipeline something 
     * that doesn't use the green lights so we don't blind everybody.
     */
    public static final int kDefaultPipeline = kAprilTagPipeline;
  }

  public static final class IntakePivotConstants {
    public static final int kMotorCanID = 15; 
    public static final int KOtherMotorCanID = 13; 

    public static final int kMotorSmartCurrentLimit = 30; // Change

    // This will invert both motors at the same time
    public static final boolean kInvertMotors = false;

    public static final boolean kTurningMotorEncoderInverted = false;
    public static final int kEncoderPort = 1;
    public static final double kAngleTolerance = 5;
    public static final double kAngleOffset = -247;

    public static final double kShooterEncoderPositionFactor = 360; 

    // Safety: prevent pivot from going beyond its minimum and maximum
    public static final Rotation2d kPivotMinAngle = Rotation2d.fromDegrees(-126.7);
    public static final Rotation2d kPivotMaxAngle = Rotation2d.fromDegrees(14.7);

    public static final double kPivotSpeed = 1;

    /*           Constants for pivot positions:            */

    public static final double kIntakeDownPosition = -126.7;
    public static final double kIntakeDeckPosition = -68;
    public static final double kIntakeInPosition = -25;
  }

  public static final class HangingConstants {
    public static final int kRightArmCanID = 18; 
    public static final int kRightArmUpperLimit = 4; 
    public static final int kRightArmLowerLimit = 5; 
    public static final boolean kRightArmInverted = false; 

    public static final int kLeftArmCanID = 19; 
    public static final int kLeftArmUpperLimit = 2; 
    public static final int kLeftArmLowerLimit = 3; 
    public static final boolean kLeftArmInverted = true; 

    public static final double kPitchP = 0.05;
    public static final double kPitchI = 0;
    public static final double kPitchD = 0.001;

    public static final int kHangingMotorCurrentLimit = 35;

  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorCanID = 14; 

    public static final int kDigitalSensorPin = 7;
    public static final boolean kSensorInverted = true;

    // Positive intakes the piece, negative retracts the piece
    public static final boolean kIntakeMotorInverted = true; 
    public static final int kIntakeMotorCurrentLimit = 35; // Change

    public static final double kHumanPlayerIntakeSpeed = .8; //oh noes sp
    public static final double kIntakeSpeed = 1; //Speed used for intaking notes
  }

  public static final class ShooterConstants {
        
    public static final int kTopShooterMotorCanID = 16;
    public static final int kBottomShooterMotorCanID = 17;
    
    public static final IdleMode kTopShooterMotor = IdleMode.kCoast;
    public static final IdleMode kBottomShooterMotor = IdleMode.kCoast;

    public static final boolean kInvertTopMotor = true;
    public static final boolean kInvertBottomMotor = false;

    public static final int kShooterMotorCurrentLimit = 35; // amps

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // Gear ratio of shooting motors
    public static final double kDrivingMotorReduction = 1;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kShooterP = 0.015;
    public static final double kShooterI = 0;
    public static final double kShooterD = 0.01;
    public static final double kShooterFF = 1 / kDriveWheelFreeSpeedRps; 
    public static final double kShooterMinOutput = -1;
    public static final double kShooterMaxOutput = 1;

    // Speed to rev the shooter to when shooting
    public static final double kWheelRevSpeed = 30; //Meters per second
    // Idle speed that shooter defaults to
    public static final double kIdleRevSpeed = 0.3; //Precent

    
    //                ************* Super important constants for math **************

    // Offsets from pivot to end of shooter
    public static final double kShooterVerticalOffset = 1.5; // Inches
    public static final double kShooterHorizontalOffset = 5; // Inches // Change

    // Angle of depression
    public static final double kAngleFromPivotToShooter = Math.atan(kShooterVerticalOffset / kShooterHorizontalOffset);
    public static final double kDistanceFromPivotToShooter = Math.hypot(kShooterVerticalOffset, kShooterHorizontalOffset);

    // How far upwards to aim up into the speaker (in inches)
    public static final double kSpeakerAimHeightInches = 27; // Adjust as needed

  }

  public static final class ShooterPivotConstants {
    public static final int kShooterPivotMotorCanID = 11; 

    public static final int kEncoderPort = 9;
    public static final double kEncoderPeriod = 1025;

    // Positive will be tilting the pivot upwards
    public static final boolean kTurningMotorInverted = false; 
    public static final boolean kTurningMotorEncoderInverted = true;
    
    public static final double kShooterEncoderPositionFactor = 360; // radians

    // Just like with the swerve wheels, this allows us to offset the encoder by a specific amount
    public static final double kAngleOffset = 71.5 + 5.9; // Adjust as necessary

    // Safety: let's set max and min angles for the shooter pivot so we don't accidentally rotate too far in one direction
    public static final double kPivotMinAngle = 0; // Degrees
    public static final double kPivotMaxAngle = 70; // Degrees

    public static final double kAngleTolerance = 5;
    
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
    public static final int kTurningMotorCurrentLimit = 30; // amps

    public static final double kPivotP = 0.005;
    public static final double kPivotI = 0;
    public static final double kPivotD = 0;

    // Constants for sinusoidal profiling
    public static final double kGravityOffsetMultiplier = 0.045; // Max output needed for when center of mass is parallel to ground.
    public static final double kShooterRestingPoint = Math.toRadians(69.3); // Horizontal shift for when the shooter is at its resting point.

    //                ************* Super important constants for math **************

    public static final double kPivotHeightInches = 16.313; // The distance between the pivot and the ground
    public static final double kPivotCenterOffsetInches = 2.505; // The horizontal distance between the center of the robot and the pivot

    // Static pivot positions
    public static final double kShooterPivotSpeakerPosition = 61;
    public static final double kShooterPivotAmpPosition = 50;
    public static final double kShooterPivotTrapPosition = 60;
  
    public static final double kShooterPivotPoduim = 37;
    public static final double kShooterPivotDeckPosition = 12; // Position for intaking note
  }

  public static final class ShooterIndexConstants {
    public static final int kMotorCanID = 10;
    public static final int kMotorCurrentLimit = 25; 

    public static final int kDigitalSensorPin = 8; 

    // Positive is intaking
    public static final boolean kInvertMotor = false;
    public static final boolean kSensorInverted = true;
    
    public static final double kIndexIntakeSpeed = .3;
  }

  public static final class AmpFlapConstants {
    public static final int kMotorCanID = 20; // set

    // Positive should bring the game piece to the shooter
    public static final boolean kMotorInverted = false; 
    public static final int kMotorCurrentLimit = 35; // Change
    public static final double kMotorSpeed = .5;

    public static final double kP = 0.15;
    public static final double kI = 0;
    public static final double kD = 0.01;


    public static final double kDownPosition = -1;
    public static final double kAmpPosition = 0.5;
  }
}