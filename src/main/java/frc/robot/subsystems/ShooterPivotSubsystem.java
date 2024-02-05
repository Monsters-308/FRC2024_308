package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import frc.robot.Constants.ShooterPivotConstants;

import com.revrobotics.CANSparkBase;

public class ShooterPivotSubsystem extends SubsystemBase {
  private final CANSparkMax shooterPivotMotor = new CANSparkMax(ShooterPivotConstants.kShooterPivotMotor, MotorType.kBrushed);
  private final ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");
  private final AbsoluteEncoder shooterPivotMotorEncoder;
  private final SparkPIDController shooterPivotMotorPIDController;

  /** Creates a new ShooterPivotSubsystem. */
  public ShooterPivotSubsystem() {
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    shooterPivotMotor.restoreFactoryDefaults();
    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    shooterPivotMotorEncoder = shooterPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shooterPivotMotorPIDController = shooterPivotMotor.getPIDController();
    shooterPivotMotorPIDController.setFeedbackDevice(shooterPivotMotorEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    shooterPivotMotorEncoder.setPositionConversionFactor(ShooterPivotConstants.kTurningEncoderPositionFactor);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    // no PID wrapping for a linear motion...?
    shooterPivotMotorPIDController.setPositionPIDWrappingEnabled(false);
    shooterPivotMotorPIDController.setPositionPIDWrappingMinInput(ShooterPivotConstants.kTurningEncoderPositionPIDMinInput);
    shooterPivotMotorPIDController.setPositionPIDWrappingMaxInput(ShooterPivotConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    shooterPivotMotorPIDController.setP(ShooterPivotConstants.kTurningP);
    shooterPivotMotorPIDController.setI(ShooterPivotConstants.kTurningI);
    shooterPivotMotorPIDController.setD(ShooterPivotConstants.kTurningD);
    shooterPivotMotorPIDController.setFF(ShooterPivotConstants.kTurningFF);
    shooterPivotMotorPIDController.setOutputRange(ShooterPivotConstants.kTurningMinOutput,
        ShooterPivotConstants.kTurningMaxOutput);

    shooterPivotMotor.setIdleMode(ShooterPivotConstants.kTurningMotorIdleMode);
    shooterPivotMotor.setSmartCurrentLimit(ShooterPivotConstants.kTurningMotorCurrentLimit);

    // Inverting turning motor
    shooterPivotMotor.setInverted(true);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    shooterPivotMotor.burnFlash();


  }

    /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public double getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return shooterPivotMotorEncoder.getPosition();
  }

  public void setPosition(double angle) {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    shooterPivotMotorPIDController.setReference(angle, CANSparkBase.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
