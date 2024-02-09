package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterPivotConstants;

import com.revrobotics.CANSparkBase;

public class ShooterPivotSubsystem extends SubsystemBase {

  private final CANSparkMax shooterPivotMotor = new CANSparkMax(ShooterPivotConstants.kShooterPivotMotorCanID, MotorType.kBrushed);
  private final AbsoluteEncoder shooterPivotMotorEncoder;
  private final SparkPIDController shooterPivotMotorPIDController;

  private final ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

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

    // Set the PID gains for the turning motor.
    shooterPivotMotorPIDController.setP(ShooterPivotConstants.kTurningP);
    shooterPivotMotorPIDController.setI(ShooterPivotConstants.kTurningI);
    shooterPivotMotorPIDController.setD(ShooterPivotConstants.kTurningD);
    shooterPivotMotorPIDController.setFF(ShooterPivotConstants.kTurningFF);
    shooterPivotMotorPIDController.setOutputRange(ShooterPivotConstants.kTurningMinOutput,
        ShooterPivotConstants.kTurningMaxOutput);

    shooterPivotMotor.setIdleMode(ShooterPivotConstants.kTurningMotorIdleMode);
    shooterPivotMotor.setSmartCurrentLimit(ShooterPivotConstants.kTurningMotorCurrentLimit);

    // Inverting turning motor
    shooterPivotMotor.setInverted(ShooterPivotConstants.kTurningMotorInverted);

    // Inverting turning encoder
    shooterPivotMotorEncoder.setInverted(ShooterPivotConstants.kTurningMotorEncoderInverted);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    shooterPivotMotor.burnFlash();

    pivotTab.addDouble("Pivot Angle", () -> getPosition());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module (in radians).
   */
  public double getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return shooterPivotMotorEncoder.getPosition();
  }

  /**
   * Set the shooter to a specific angle (in radians)
   * @param angle The angle to set the shooter to (in radians)
   */
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
