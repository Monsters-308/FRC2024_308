package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;

public class IndexSubsystem extends SubsystemBase {

  private final CANSparkMax m_indexMotor = new CANSparkMax(IndexConstants.kIndexMotorCanID, MotorType.kBrushless);
    
  /** Creates a new IndexSubsystem. */
  public IndexSubsystem() {
    // Restore the motor controller to a known state (in case it's swapped out)
    m_indexMotor.restoreFactoryDefaults();

    // Limit the current so the motor doesn't oof itself
    m_indexMotor.setSmartCurrentLimit(IndexConstants.kIndexMotorCurrentLimit);

    // Set to coast mode
    m_indexMotor.setIdleMode(IdleMode.kCoast);

    // Set inverted
    m_indexMotor.setInverted(IndexConstants.kIndexMotorInverted);

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
