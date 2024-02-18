package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivotSubsystem extends SubsystemBase {

  // Variable to represent the state of the intake: false is up, true is down
  // I would turn this into an enum but it's only 2 states
  private boolean m_intakeState = false;
  
  private final CANSparkMax m_masterMotor = new CANSparkMax(IntakePivotConstants.kMotorCanID, MotorType.kBrushed);
  private final CANSparkMax m_followerMotor = new CANSparkMax(IntakePivotConstants.KOtherMotorCanID, MotorType.kBrushed);

  private final DigitalInput m_upperLimit = new DigitalInput(IntakePivotConstants.kUpperLimitPort);
  private final DigitalInput m_lowerLimit = new DigitalInput(IntakePivotConstants.kLowerLimitPort);

  /** Creates a new IntakePivotSubsystem. */
  public IntakePivotSubsystem() {
    // Reset the motor controller to a known state
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
    m_followerMotor.follow(m_masterMotor);

    // Save the motor controller's configuration
    m_masterMotor.burnFlash();
    m_followerMotor.burnFlash();
  }

  /** Set speed of pivot (for testing purposes) */
  public void setSpeed(double speed){
    m_masterMotor.set(speed);
  }

  /** Returns true if the intake is fully up and activating the upper limit switch. */
  public boolean isIntakeUp(){
    return m_upperLimit.get();
  }

  /** Returns true if the intake is fully down and activating the lower limit switch. */
  public boolean isIntakeDown(){
    return m_lowerLimit.get();
  }

  /** Makes the intake move down until it is touching the lower limit switch. */
  public void intakeDown(){
    m_intakeState = true;
  }

  /** Makes the intake move up until it is touching the upper limit switch. */
  public void intakeUp(){
    m_intakeState = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    manageState();
  }

  /** I made a separate function for this so we can more easily comment it out during testing. */
  private void manageState(){
    // Run motor forward if not down
    if (m_intakeState && !isIntakeDown()){
      m_masterMotor.set(IntakePivotConstants.kIntakeDownSpeed);
    }
    // Run motor backwards if not up
    else if (!m_intakeState && !isIntakeUp()){
      m_masterMotor.set(IntakePivotConstants.kIntakeUpSpeed);
    }
    // Stop movement otherwise
    else {
      m_masterMotor.set(0);
    }
  }

}
