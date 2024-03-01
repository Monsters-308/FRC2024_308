package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpFlapConstants;

public class AmpFlapSubsystem extends SubsystemBase {

  private final CANSparkMax m_indexMotor = new CANSparkMax(AmpFlapConstants.kMotorCanID, MotorType.kBrushed);
  private final SparkPIDController m_PidController = m_indexMotor.getPIDController();
  private final PIDController m_PidController2 = new PIDController(AmpFlapConstants.kP, 
                                                                    AmpFlapConstants.kI, 
                                                                    AmpFlapConstants.kD);
    
  /** Creates a new IndexSubsystem. */
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

    m_PidController2.setSetpoint(AmpFlapConstants.kDownPosition);

    Shuffleboard.getTab("Flap Thingy").addDouble("Flap position", () -> getPosition());

    //Shuffleboard.getTab("Flap Thingy").add("Reset Encoder", new InstantCommand(() -> m_motorEncoder.setPosition(0)).ignoringDisable(true));
  }

  /**
   * Sets the motor to a certain speed.
   * @param speed A speed from -1 to 1.
   */
  public void setSpeed(double speed){
    m_indexMotor.set(speed);
  }

  /**
   * Get the position of the relative encoder
   * @return
   */
  public double getPosition(){
    return 0;//m_motorEncoder.getPosition();
  }

  /**
   * Set the PID controller to a certain position
   * @param position
   */
  public void setPosition(double position){
    m_PidController2.setSetpoint(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //setSpeed(m_PidController2.calculate(getPosition()));
  }

}
