package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.intake.RunIntake;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanID, MotorType.kBrushed);
  private final DigitalInput m_noteSensor = new DigitalInput(IntakeConstants.kDigitalSensorPin);
    
  /**
   * Controls just the motor that spins the intake rollers, as well as the light sensor 
   * to detect if a note is in the intake. 
   */
  public IntakeSubsystem() {
    // Restore the motor controller to a known state (in case it's swapped out)
    m_intakeMotor.restoreFactoryDefaults();

    // Limit the current so the motor doesn't oof itself
    m_intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

    // Set to coast mode
    m_intakeMotor.setIdleMode(IdleMode.kBrake);

    // Set inverted
    m_intakeMotor.setInverted(IntakeConstants.kIntakeMotorInverted);

    // Save the motor controller's configuration
    m_intakeMotor.burnFlash();

    Shuffleboard.getTab("Intake").addBoolean("Game Piece", () -> !m_noteSensor.get());

    // Widget for testing intake motor
    Shuffleboard.getTab("Intake").add("Run Intake", new RunIntake(this, 1));
  }

  /**
   * Returns whether or not there is a game piece in the intake.
   * @return true if there's a game piece.
   */
  public boolean gamePieceDetected(){
    if (IntakeConstants.kSensorInverted){
      return !m_noteSensor.get();
    }
    return m_noteSensor.get();
  }  

  /**
   * Sets the motor to a certain speed.
   * @param speed A speed from -1 to 1.
   */
  public void setSpeed(double speed){
    m_intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
