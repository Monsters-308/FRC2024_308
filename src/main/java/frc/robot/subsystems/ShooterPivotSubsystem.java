package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterPivotConstants;

public class ShooterPivotSubsystem extends SubsystemBase {

  private final CANSparkMax m_shooterPivotMotor = new CANSparkMax(ShooterPivotConstants.kShooterPivotMotorCanID, MotorType.kBrushed);
  private final DutyCycleEncoder m_shooterPivotMotorEncoder = new DutyCycleEncoder(ShooterPivotConstants.kEncoderPort);
  private Rotation2d m_desiredAngle = new Rotation2d();


  private final ShuffleboardTab pivotTab = Shuffleboard.getTab("Shooter");

  /** Creates a new ShooterPivotSubsystem. */
  public ShooterPivotSubsystem() {
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_shooterPivotMotor.restoreFactoryDefaults();
    
    // Conversion factor from default unit of rotations to RADIANS
    //returns a distance of kShooterEncoderPositionFactor for every rotation
    m_shooterPivotMotorEncoder.setDistancePerRotation(Math.PI * 2);
    //m_shooterPivotMotorEncoder.setDistancePerRotation(0.5); TEST
    // Set DutyCycle range (Note: This should fix everything but)
    //what would happen if we dont set the value??
    m_shooterPivotMotorEncoder.setDutyCycleRange(
        1 / ShooterPivotConstants.kEncoderPeriod, 
        (ShooterPivotConstants.kEncoderPeriod-1) / ShooterPivotConstants.kEncoderPeriod);

    //m_shooterPivotMotorEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0/1025.0); TEST
    //what happens if we get the raw data and multiply it by 360?
        
    // More motor configuration
    m_shooterPivotMotor.setIdleMode(ShooterPivotConstants.kTurningMotorIdleMode);
    m_shooterPivotMotor.setSmartCurrentLimit(ShooterPivotConstants.kTurningMotorCurrentLimit);

    // Inverting turning motor
    m_shooterPivotMotor.setInverted(ShooterPivotConstants.kTurningMotorInverted);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_shooterPivotMotor.burnFlash();

    pivotTab.addDouble("Pivot Angle", () -> getPosition().getDegrees());
  }

  /**
   * Returns the angle the shooter is at.
   * The angle is 0 when the shooter is parallel to the ground.
   *
   * @return The current position of the module (in radians).
   */
  public Rotation2d getPosition() {
    double angleDegrees;

    if(ShooterPivotConstants.kTurningMotorEncoderInverted){
      angleDegrees = Math.toDegrees(-m_shooterPivotMotorEncoder.getAbsolutePosition() + ShooterPivotConstants.kAngleOffset);
    }
    else {
      angleDegrees = Math.toDegrees(m_shooterPivotMotorEncoder.getAbsolutePosition() + ShooterPivotConstants.kAngleOffset);
    }
    
    // Stupid conversion math
    return Rotation2d.fromDegrees(((angleDegrees - 30.5) * 6.3659) + 30.5);
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
    m_desiredAngle = Rotation2d.fromDegrees(angle);
  }

  /**
   * Set the shooter to a specific angle 
   * @param angle The angle to set the shooter to 
   */
  public void setPosition(Rotation2d angle) {
    m_desiredAngle = angle;
  }


  /**
   * Returns whether the shooter pivot is at its desired position within an amount of tolerance.
   * @return true if in its desired position.
   */
  public boolean inPosition(){
    double currentAngleDegrees = getPosition().getDegrees();
    double desiredAngleDegrees = m_desiredAngle.getDegrees();

    return Math.abs(currentAngleDegrees - desiredAngleDegrees) > ShooterPivotConstants.kAngleTolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("ShooterPivot Connected", m_shooterPivotMotorEncoder.isConnected());
    SmartDashboard.putNumber("ShooterPivot value", m_shooterPivotMotorEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("ShooterPivot Radians", m_shooterPivotMotorEncoder.getAbsolutePosition()* 2.0 * Math.PI);
    manageState();    
  }

  /** Putting this in a separate function so we can comment it out easier */
  private void manageState(){

    double currentAngleDegrees = getPosition().getDegrees();
    double desiredAngleDegrees = m_desiredAngle.getDegrees();

    if(Math.abs(currentAngleDegrees - desiredAngleDegrees) > ShooterPivotConstants.kAngleTolerance){
      if(currentAngleDegrees > desiredAngleDegrees){
        m_shooterPivotMotor.set(-1);
      }
      else if (currentAngleDegrees < desiredAngleDegrees){
        m_shooterPivotMotor.set(1);
      }
    }
    else {
      m_shooterPivotMotor.set(0);
    }
  }

}
