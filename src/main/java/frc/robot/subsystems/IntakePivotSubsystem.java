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
  
  private final CANSparkMax m_motor = new CANSparkMax(IntakePivotConstants.kMotorCanID, MotorType.kBrushed);
  private final CANSparkMax m_otherMotor = new CANSparkMax(IntakePivotConstants.kMotorCanID+1, MotorType.kBrushed);
  private final DigitalInput m_upperLimit = new DigitalInput(IntakePivotConstants.kUpperLimitPort);
  private final DigitalInput m_lowerLimit = new DigitalInput(IntakePivotConstants.kLowerLimitPort);

  /** Creates a new ArmSubsystem. */
  public IntakePivotSubsystem() {
    // Reset the motor controller to a known state
    m_motor.restoreFactoryDefaults();

    // Limit the current so the motor doesn't oof itself
    m_motor.setSmartCurrentLimit(IntakePivotConstants.kMotorSmartCurrentLimit);

    // Set to brake mode
    m_motor.setIdleMode(IdleMode.kBrake);

    // Set inverted (positive should be down)
    m_motor.setInverted(IntakePivotConstants.kInvertMotor);

    // Save the motor controller's configuration
    m_motor.burnFlash();
  }

  /** Set speed of pivot (for testing purposes) */
  public void setSpeed(double speed){
    m_motor.set(speed);
  }

  public boolean isIntakeUp(){
    return m_upperLimit.get();
  }

  public boolean isIntakeDown(){
    return m_lowerLimit.get();
  }

  public void intakeDown(){
    m_intakeState = true;
  }

  public void intakeUp(){
    m_intakeState = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Run motor forward if not down
    if (m_intakeState && !isIntakeDown()){
      m_motor.set(0.5);
    }
    // Run motor backwards if not up
    else if (!m_intakeState && !isIntakeUp()){
      m_motor.set(-0.5);
    }
    // Stop movement otherwise
    else {
      m_motor.set(0);
    }

  }

}
