package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterPivotConstants;
import frc.utils.SwerveUtils;

public class ShooterPivotSubsystem extends SubsystemBase {

  private final CANSparkMax m_shooterPivotMotor = new CANSparkMax(ShooterPivotConstants.kShooterPivotMotorCanID, MotorType.kBrushed);

  private final DutyCycleEncoder m_shooterPivotMotorEncoder = new DutyCycleEncoder(ShooterPivotConstants.kEncoderPort);

  private double m_desiredAngleDegrees = ShooterPivotConstants.kShooterPivotSpeakerPosition;

  private final PIDController m_angleController = new PIDController(ShooterPivotConstants.kPivotP, 
                                                                    ShooterPivotConstants.kPivotI, 
                                                                    ShooterPivotConstants.kPivotD);

  private final ShuffleboardTab pivotTab = Shuffleboard.getTab("Shooter");

  /** 
   * Manages the angle of the shooter using a linear actuator to rotate the shooter up and 
   * down and using an absolute encoder to track the angle of the shooter.
   */
  public ShooterPivotSubsystem() {
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_shooterPivotMotor.restoreFactoryDefaults();

    // Set DutyCycle range 
    m_shooterPivotMotorEncoder.setDutyCycleRange(
        1 / ShooterPivotConstants.kEncoderPeriod, 
        (ShooterPivotConstants.kEncoderPeriod-1) / ShooterPivotConstants.kEncoderPeriod);
        
    // More motor configuration
    m_shooterPivotMotor.setIdleMode(ShooterPivotConstants.kTurningMotorIdleMode);
    m_shooterPivotMotor.setSmartCurrentLimit(ShooterPivotConstants.kTurningMotorCurrentLimit);

    // Inverting turning motor
    m_shooterPivotMotor.setInverted(ShooterPivotConstants.kTurningMotorInverted);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_shooterPivotMotor.burnFlash();

    pivotTab.addDouble("Pivot Angle", () -> getPosition().getDegrees());

    pivotTab.addBoolean("Encoder Connected", () -> m_shooterPivotMotorEncoder.isConnected());

    pivotTab.add("Speaker Position", new InstantCommand(() -> setPosition(ShooterPivotConstants.kShooterPivotSpeakerPosition)));
    
    pivotTab.add("Pivot Down", new InstantCommand(() -> setPosition(0)));
  }

  /**
   * Returns the angle the shooter is at.
   * The angle is 0 when the shooter is parallel to the ground.
   *
   * @return The current position of the module.
   */
  public Rotation2d getPosition() {
    return Rotation2d.fromDegrees(
      SwerveUtils.angleConstrain(
        (ShooterPivotConstants.kTurningMotorEncoderInverted ? -1 : 1) * 
        m_shooterPivotMotorEncoder.getAbsolutePosition() * ShooterPivotConstants.kShooterEncoderPositionFactor + ShooterPivotConstants.kAngleOffset
      ) 
    );
  }

  /** This function is for testing purposes */
  public void setSpeed(double speed){
    m_shooterPivotMotor.set(speed);
  }

  /**
   * Set the shooter to a specific angle (in degrees)
   * @param angle The angle to set the shooter to (in degrees)
   */
  public void setPosition(double angle) {
    m_desiredAngleDegrees = MathUtil.clamp(angle, ShooterPivotConstants.kPivotMinAngle, ShooterPivotConstants.kPivotMaxAngle); 
  }

  /**
   * Set the shooter to a specific angle 
   * @param angle The angle to set the shooter to 
   */
  public void setPosition(Rotation2d angle) {
    m_desiredAngleDegrees = MathUtil.clamp(angle.getDegrees(), ShooterPivotConstants.kPivotMinAngle, ShooterPivotConstants.kPivotMaxAngle);
  }

  /**
   * Returns whether the shooter pivot is at its desired position within an amount of tolerance.
   * @return true if in its desired position.
   */
  public boolean inPosition(){
    double currentAngleDegrees = getPosition().getDegrees();
    
    return Math.abs(currentAngleDegrees - m_desiredAngleDegrees) < ShooterPivotConstants.kAngleTolerance;
  }

  /** Stops pivot movement by setting the desired angle to the current angle. */
  public void stopMovement(){
    m_desiredAngleDegrees = getPosition().getDegrees();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    manageState();    
  }

  /** Putting this in a separate function so we can comment it out easier */
  private void manageState() {
    m_angleController.setSetpoint(m_desiredAngleDegrees);

    double currentAngleDegrees = getPosition().getDegrees();
    
    double currentAngleRadians = getPosition().getRadians();
    
    // Sinusoidal profiling to adjust for gravity
    double gravityOffset = Math.cos(currentAngleRadians + ShooterPivotConstants.kShooterRestingPoint) * ShooterPivotConstants.kGravityOffsetMultiplier;

    // Total motor output with PID and gravity adjustment
    double totalMotorOutput = m_angleController.calculate(currentAngleDegrees) + gravityOffset;

    m_shooterPivotMotor.set(totalMotorOutput);
  }
}