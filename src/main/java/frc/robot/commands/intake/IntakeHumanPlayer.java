package frc.robot.commands.intake;

import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.Constants.ShooterIndexConstants;
    
public class IntakeHumanPlayer extends Command {
  private final ShooterPivotSubsystem m_shooterPivotSubsystem; 
  private final ShooterIndexSubsystem m_shooterIndexSubsystem;

  public IntakeHumanPlayer(ShooterPivotSubsystem shooterPivotSubsystem, ShooterIndexSubsystem shooterIndexSubsystem) {
    m_shooterPivotSubsystem = shooterPivotSubsystem; 
    m_shooterIndexSubsystem = shooterIndexSubsystem;
    addRequirements(shooterPivotSubsystem, shooterIndexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    if (m_shooterPivotSubsystem.inPosition()){
      m_shooterIndexSubsystem.setSpeed(ShooterIndexConstants.kIndexIntakeSpeed);
    }

    else {
      m_shooterIndexSubsystem.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterIndexSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooterIndexSubsystem.gamePieceDetected();
  }
}
