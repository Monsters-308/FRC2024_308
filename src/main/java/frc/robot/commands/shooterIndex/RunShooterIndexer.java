package frc.robot.commands.shooterIndex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIndexSubsystem;

public class RunShooterIndexer extends Command {
  private final ShooterIndexSubsystem m_shooterIndexSubsystem;
  private final double m_speed;

  /**
   * Runs the shooter indexer motor at a specified speed.
   * NOTE: this is equivalent to new InstantCommand(() -> m_shooterIndexSubsystem.setSpeed(0), shooterIndexSubsystem) 
   * except it stops the motor when interrupted.
   * @param indexSubsystem
   */
  public RunShooterIndexer(ShooterIndexSubsystem indexSubsystem, double speed) {
    m_shooterIndexSubsystem = indexSubsystem;
    m_speed = speed;

    addRequirements(m_shooterIndexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterIndexSubsystem.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterIndexSubsystem.setSpeed(0);
  }

}
