package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivotSubsystem extends SubsystemBase {

  private final CANSparkMax m_masterMotor = new CANSparkMax(IntakePivotConstants.kMotorCanID, MotorType.kBrushed);
  private final CANSparkMax m_followerMotor = new CANSparkMax(IntakePivotConstants.KOtherMotorCanID, MotorType.kBrushed);

  private final DutyCycleEncoder m_intakePivotMotorEncoder = new DutyCycleEncoder(IntakePivotConstants.kEncoderPort);

  private double m_desiredAngle = IntakePivotConstants.kPivotMinAngle;

  private final PIDController pidController = new PIDController(IntakePivotConstants.kPivotP, 
                                                                IntakePivotConstants.kPivotI, 
                                                                IntakePivotConstants.kPivotD);

  /** Creates a new IntakePivotSubsystem. */
  public IntakePivotSubsystem() {
    m_intakePivotMotorEncoder.setDistancePerRotation(IntakePivotConstants.kShooterEncoderPositionFactor);
    
    pidController.setTolerance(IntakePivotConstants.kAngleTolerance);

    pidController.reset();
    pidController.setSetpoint(m_desiredAngle);

    m_masterMotor.restoreFactoryDefaults();
    m_followerMotor.restoreFactoryDefaults();

    // Limit the current so the motor doesn't oof itself
    m_masterMotor.setSmartCurrentLimit(IntakePivotConstants.kMotorSmartCurrentLimit);
    m_followerMotor.setSmartCurrentLimit(IntakePivotConstants.kMotorSmartCurrentLimit);

    // Set to brake mode
    m_masterMotor.setIdleMode(IdleMode.kBrake);
    m_followerMotor.setIdleMode(IdleMode.kBrake);

    // Set inverted (positive should be down)
    m_masterMotor.setInverted(IntakePivotConstants.kInvertMotors);
    m_followerMotor.setInverted(!IntakePivotConstants.kInvertMotors);

    // Have motors use the same can ID because it's technically slightly more efficient
    m_followerMotor.follow(m_masterMotor, true);

    // Save the motor controller's configuration
    m_masterMotor.burnFlash();
    m_followerMotor.burnFlash();
  }

  /**
   * Returns the angle the pivot is at.
   * The angle is 0 when the pivot is parallel to the ground.
   *
   * @return The current position of the module (in radians).
   */
  public double getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    if(IntakePivotConstants.kTurningMotorEncoderInverted){
      return -m_intakePivotMotorEncoder.getAbsolutePosition() + IntakePivotConstants.kAngleOffset;
    }
    return m_intakePivotMotorEncoder.getAbsolutePosition() + IntakePivotConstants.kAngleOffset;
  
  }

  /** This function is for testing purposes */
  public void setSpeed(double speed){
     m_masterMotor.set(speed);
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
    pidController.setSetpoint(m_desiredAngle);
    double rotation = pidController.calculate(getPosition());

    m_masterMotor.set(rotation);
  }

}
