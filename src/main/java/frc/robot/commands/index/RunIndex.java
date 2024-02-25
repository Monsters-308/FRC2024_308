package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;

public class RunIndex extends Command {
  private final IndexSubsystem m_indexSubsystem;
  private final double m_speed;

  /**
   * Runs the indexer (hotdog roller) at a specified speed.
   * NOTE: this is equivalent to new InstantCommand(() -> m_indexSubsystem.setSpeed(speed), m_indexSubsystem)
   * except it stops the motor when interrupted.
   * @param indexSubsystem
   */
  public RunIndex(IndexSubsystem indexSubsystem, double speed) {
    m_indexSubsystem = indexSubsystem;
    m_speed = speed;

    addRequirements(m_indexSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexSubsystem.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexSubsystem.setSpeed(0);
  }

}
