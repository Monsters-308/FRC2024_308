package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterPivotConstants;

public class ShooterPivotSubsystem extends SubsystemBase {

  private final CANSparkMax m_shooterPivotMotor = new CANSparkMax(ShooterPivotConstants.kShooterPivotMotorCanID, MotorType.kBrushed);
  private final DutyCycleEncoder m_shooterPivotMotorEncoder = new DutyCycleEncoder(ShooterPivotConstants.kEncoderPort);
  private double m_desiredAngle = ShooterPivotConstants.kPivotMinAngle;


  private final ShuffleboardTab pivotTab = Shuffleboard.getTab("Shooter");

  /** Creates a new ShooterPivotSubsystem. */
  public ShooterPivotSubsystem() {
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_shooterPivotMotor.restoreFactoryDefaults();
    
    // Conversion factor from default unit of rotations
    m_shooterPivotMotorEncoder.setDistancePerRotation(ShooterPivotConstants.kShooterEncoderPositionFactor);
    // Set DutyCycle range
    m_shooterPivotMotorEncoder.setDutyCycleRange(
        1 / ShooterPivotConstants.kEncoderPeriod, 
        ShooterPivotConstants.kEncoderPeriod-1 / ShooterPivotConstants.kEncoderPeriod);
    
    m_shooterPivotMotor.setIdleMode(ShooterPivotConstants.kTurningMotorIdleMode);
    m_shooterPivotMotor.setSmartCurrentLimit(ShooterPivotConstants.kTurningMotorCurrentLimit);

    // Inverting turning motor
    m_shooterPivotMotor.setInverted(ShooterPivotConstants.kTurningMotorInverted);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_shooterPivotMotor.burnFlash();

    pivotTab.addDouble("Pivot Angle", () -> Math.toDegrees(getPosition()));
  }

  /**
   * Returns the angle the shooter is at.
   * The angle is 0 when the shooter is parallel to the ground.
   *
   * @return The current position of the module (in radians).
   */
  public double getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    if(ShooterPivotConstants.kTurningMotorEncoderInverted){
      return -m_shooterPivotMotorEncoder.getAbsolutePosition() + ShooterPivotConstants.kAngleOffset;
    }
    return m_shooterPivotMotorEncoder.getAbsolutePosition() + ShooterPivotConstants.kAngleOffset;
  
  }

  public void setSpeed(double speed){
     m_shooterPivotMotor.set(speed);
  }

  /**
   * Set the shooter to a specific angle (in radians)
   * @param angle The angle to set the shooter to (in radians)
   */
  public void setPosition(double angle) {
    m_desiredAngle = angle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //manageState();    
  }

  /** Putting this in a separate function so we can comment it out easier */
  private void manageState(){

    if(Math.abs(getPosition()-m_desiredAngle) > ShooterPivotConstants.kAngleTolerance){
      if(getPosition() > m_desiredAngle){
        m_shooterPivotMotor.set(-1);
      }
      else if (getPosition() < m_desiredAngle){
        m_shooterPivotMotor.set(1);
      }
    }
    else {
      m_shooterPivotMotor.set(0);
    }
  }

}
