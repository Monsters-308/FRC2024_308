package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivotSubsystem extends SubsystemBase {
  
  private final CANSparkMax m_leftMotor = new CANSparkMax(IntakePivotConstants.kLeftMotorCanID, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor = new CANSparkMax(IntakePivotConstants.kRightMotorCanID, MotorType.kBrushless);


  /** Creates a new ArmSubsystem. */
  public IntakePivotSubsystem() {
    m_leftMotor.restoreFactoryDefaults();

    // Limit the current so the motor doesn't oof itself
    m_leftMotor.setSmartCurrentLimit(IndexConstants.kIndexMotorCurrentLimit);

    // Set to coast mode
    m_leftMotor.setIdleMode(IdleMode.kBrake);

    // Set inverted
    m_leftMotor.setInverted(false);

    // Save the motor controller's configuration
    m_leftMotor.burnFlash();

    m_leftMotor.restoreFactoryDefaults();

    // Limit the current so the motor doesn't oof itself
    m_rightMotor.setSmartCurrentLimit(IndexConstants.kIndexMotorCurrentLimit);

    // Set to coast mode
    m_rightMotor.setIdleMode(IdleMode.kBrake);

    // Set inverted
    m_rightMotor.setInverted(true);

    // Save the motor controller's configuration
    m_rightMotor.burnFlash();
  }

  /** Set speed of left arm */
  public void setLeftSpeed(double speed){
    m_leftMotor.set(speed);
  }

  /** Set speed of right arm */
  public void setRightSpeed(double speed){
    m_rightMotor.set(speed);
  }

  /** Set speed of both arms */
  public void setBothSpeed(double speed){
    m_leftMotor.set(speed);
    m_rightMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
