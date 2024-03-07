package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpFlapConstants;

public class AmpFlapSubsystem extends SubsystemBase {

  private final CANSparkMax m_indexMotor = new CANSparkMax(AmpFlapConstants.kMotorCanID, MotorType.kBrushed);
    
  /** This controls the motor on top of our robot that rotates the amp flap. */
  public AmpFlapSubsystem() {
    // Restore the motor controller to a known state (in case it's swapped out)
    m_indexMotor.restoreFactoryDefaults();

    // Limit the current so the motor doesn't oof itself
    m_indexMotor.setSmartCurrentLimit(AmpFlapConstants.kMotorCurrentLimit);

    // Set to brake mode
    m_indexMotor.setIdleMode(IdleMode.kBrake);

    // Set inverted
    m_indexMotor.setInverted(AmpFlapConstants.kMotorInverted);

    // Save the motor controller's configuration
    m_indexMotor.burnFlash();
  }

  /**
   * Sets the motor to a certain speed.
   * @param speed A speed from -1 to 1.
   */
  public void setSpeed(double speed){
    m_indexMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
