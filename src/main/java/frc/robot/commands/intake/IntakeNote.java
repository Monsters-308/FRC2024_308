package frc.robot.commands.intake;

import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IndexConstants;

public class IntakeNote extends Command {
  private final IntakeSubsystem m_intakeSubsystem;;
  private final IntakePivotSubsystem m_intakePivotSubsystem;

   /**
    * Pivots the intake down and runs the intake roller until a note is collected
    * @param intakeSubsystem
    * @param shooterPivotSubsystem
    * @param shooterIndexSubsystem
    * @param indexSubsystem
    * @return gamePieceDetected (when the light sensor detects a note in the intake)
    */
  public IntakeNote(IntakeSubsystem intakeSubsystem, IntakePivotSubsystem intakePivotSubsystem) {
    m_intakeSubsystem = intakeSubsystem; 
    m_intakePivotSubsystem = intakePivotSubsystem;

    addRequirements(intakeSubsystem, intakePivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //gets a note in the intake
    m_intakePivotSubsystem.setPosition(IntakeConstants.kIntakePivotDownPosition);
    
    if (m_intakePivotSubsystem.getPosition() == IntakeConstants.kIntakePivotDownPosition){
      m_intakeSubsystem.setSpeed(IndexConstants.kIndexIntakeSpeed);
    }
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
