package frc.robot.commands.intake;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.ShooterIndexConstants;
    
public class IntakeHumanPlayer extends Command {
  private final ShooterPivotSubsystem m_shooterPivotSubsystem; 
  private final ShooterIndexSubsystem m_shooterIndexSubsystem;
  private final IndexSubsystem m_hotdogRoller;

  public IntakeHumanPlayer(ShooterPivotSubsystem shooterPivotSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, 
                           IndexSubsystem indexSubsystem) {
    m_shooterPivotSubsystem = shooterPivotSubsystem; 
    m_hotdogRoller = indexSubsystem;
    m_shooterIndexSubsystem = shooterIndexSubsystem;
    addRequirements(shooterPivotSubsystem, indexSubsystem, shooterIndexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    if (m_shooterPivotSubsystem.inPosition()){
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooterIndexSubsystem.gamePieceDetected();
  }
}
