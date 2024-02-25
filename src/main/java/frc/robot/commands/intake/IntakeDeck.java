package frc.robot.commands.intake;

import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakePivotConstants;
    
public class IntakeDeck extends Command {

  private final IntakeSubsystem m_intakeSubsystem;  
  private final ShooterPivotSubsystem m_shooterPivotSubsystem; 
  private final IndexSubsystem m_hotdogRoller;
  private final IntakePivotSubsystem m_intakePivotSubsystem;

   /**
    * Pivots the intake into deck position and launches any note in the intake onto the hot dog roller. The deck rolls the note into the shooter loading the note for shooting.
    * The intake goes into the upwards position after the note successfully indexes 
    * @param intakeSubsystem
    * @param shooterPivotSubsystem
    * @param indexSubsystem
    */
  public IntakeDeck(IntakeSubsystem intakeSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, 
                           IndexSubsystem indexSubsystem, IntakePivotSubsystem intakePivotSubsystem) {
    m_intakeSubsystem = intakeSubsystem; 
    m_shooterPivotSubsystem = shooterPivotSubsystem; 
    m_hotdogRoller = indexSubsystem;
    m_intakePivotSubsystem = intakePivotSubsystem;

    addRequirements(intakeSubsystem, shooterPivotSubsystem, indexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //gets a note on the deck
    m_intakePivotSubsystem.setPosition(IntakePivotConstants.kIntakeDeckPostion);
    if (m_intakePivotSubsystem.getPosition() == IntakePivotConstants.kIntakeDeckPostion){
      m_intakeSubsystem.setSpeed(IndexConstants.kIndexIntakeSpeed);
    }
    //gets a note in the shooter
    if (m_shooterPivotSubsystem.inPosition()){
      m_intakeSubsystem.setSpeed(IntakeConstants.kIntakeSpeed);
      m_hotdogRoller.setSpeed(IndexConstants.kIndexIntakeSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hotdogRoller.setSpeed(0);
    m_intakeSubsystem.setSpeed(0);
    m_intakePivotSubsystem.setPosition(IntakePivotConstants.kIntakeInPosition);
  }

}
