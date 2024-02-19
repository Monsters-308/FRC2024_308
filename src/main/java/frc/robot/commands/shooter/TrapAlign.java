package frc.robot.commands.shooter;

import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class TrapAlign extends Command {
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;

  // variables for motor speeds/velocities

  public TrapAlign(ShooterPivotSubsystem pivotSubsystem) {
    m_shooterPivotSubsystem = pivotSubsystem;

    addRequirements( m_shooterPivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterPivotSubsystem.setPosition(70);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_shooterPivotSubsystem.getPosition() == 70) {
      return true;
    }
    return false;
  }
}