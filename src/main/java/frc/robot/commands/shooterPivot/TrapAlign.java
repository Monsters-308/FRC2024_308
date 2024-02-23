package frc.robot.commands.shooterPivot;

import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.subsystems.ShooterPivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class TrapAlign extends Command {
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;

  /**
   * Aims shooter pivot at amp.
   * This command only ends once the pivot is lined up with the amp.
   * @param pivotSubsystem
   */
  public TrapAlign(ShooterPivotSubsystem pivotSubsystem) {
    m_shooterPivotSubsystem = pivotSubsystem;

    addRequirements(m_shooterPivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterPivotSubsystem.setPosition(ShooterPivotConstants.kShooterPivotTrapPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_shooterPivotSubsystem.getPosition() == ShooterPivotConstants.kShooterPivotTrapPosition) {
      return true;
    }
    return false;
  }
}
