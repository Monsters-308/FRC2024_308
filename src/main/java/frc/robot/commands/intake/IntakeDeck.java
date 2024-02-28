package frc.robot.commands.intake;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IndexConstants;
    
public class IntakeDeck extends Command {

  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexSubsystem m_hotdogRoller;

   /**
    * Launches any note in the intake onto the hot dog roller. 
    * The deck rolls the note into the shooter, loading the note for shooting.
    * @param intakeSubsystem
    * @param indexSubsystem
    */
  public IntakeDeck(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {
    m_intakeSubsystem = intakeSubsystem; 
    m_hotdogRoller = indexSubsystem;

    addRequirements(intakeSubsystem, indexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //gets a note on the deck
    m_intakeSubsystem.setSpeed(IntakeConstants.kIntakeSpeed);
    m_hotdogRoller.setSpeed(IndexConstants.kIndexIntakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hotdogRoller.setSpeed(0);
    m_intakeSubsystem.setSpeed(0);
  }

}
