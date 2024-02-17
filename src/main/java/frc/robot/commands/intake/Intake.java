package frc.robot.commands.intake;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.ShooterPivotConstants; 
import frc.robot.Constants.ShooterIndexConstants;
    
public class Intake extends Command {
  private final IntakeSubsystem m_intakeSubsystem;  
  private final ShooterPivotSubsystem m_shooterPivotSubsystem; 
  private final ShooterIndexSubsystem m_shooterIndexSubsystem;
  private final IndexSubsystem m_hotdogRoller;
  //variables for motor speeds/velocities


  public Intake(IntakeSubsystem intakeSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, 
                           IndexSubsystem indexSubsystem) {
    m_intakeSubsystem = intakeSubsystem; 
    m_shooterPivotSubsystem = shooterPivotSubsystem; 
    m_hotdogRoller = indexSubsystem;
    m_shooterIndexSubsystem = shooterIndexSubsystem;
    addRequirements(intakeSubsystem, shooterPivotSubsystem, indexSubsystem, shooterIndexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    m_intakeSubsystem.setSpeed(IntakeConstants.kIntakeSpeed);
  
    if (m_shooterPivotSubsystem.getPosition() == ShooterPivotConstants.kshooterPivotDownPosition){
      m_hotdogRoller.setSpeed(IndexConstants.kIndexIntakeSpeed);
      m_shooterIndexSubsystem.setSpeed(ShooterIndexConstants.kIndexIntakeSpeed);
    }

    else {
      m_hotdogRoller.setSpeed(0);
      m_shooterIndexSubsystem.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hotdogRoller.setSpeed(0);
    m_shooterIndexSubsystem.setSpeed(0);
    m_intakeSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooterIndexSubsystem.gamePieceDetected();
  }
}
