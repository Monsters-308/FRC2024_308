package frc.robot.commands.shooterIndex;

import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
  private final ShooterIndexSubsystem m_shooterIndexSubsystem;

  // variables for motor speeds/velocities

  public IntakeNote(ShooterIndexSubsystem indexSubsystem) {
    m_shooterIndexSubsystem = indexSubsystem;

    addRequirements(m_shooterIndexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterIndexSubsystem.setSpeed(0.75);
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
