package frc.robot.commands.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 

public class IntakeNote extends Command {
  private final IntakeSubsystem m_intakeSubsystem;

   /**
    * Runs the intake roller until a note is detected by the light sensor.
    * @param intakeSubsystem
    * @return gamePieceDetected (when the light sensor detects a note in the intake)
    */
  public IntakeNote(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem; 

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.setSpeed(IntakeConstants.kIntakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.gamePieceDetected();
  }

}
