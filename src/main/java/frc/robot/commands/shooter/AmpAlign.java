package frc.robot.commands.shooter;

import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AmpAlign extends Command {
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ShooterIndexSubsystem m_shooterIndexSubsystem;
  private final DriveSubsystem m_driveSubsystem;

  // variables for motor speeds/velocities

  public AmpAlign(ShooterSubsystem Subsystem, ShooterPivotSubsystem pivotSubsystem,
      ShooterIndexSubsystem indexSubsystem, DriveSubsystem driveSubsystem) {
    m_shooterSubsystem = Subsystem;
    m_shooterPivotSubsystem = pivotSubsystem;
    m_shooterIndexSubsystem = indexSubsystem;
    m_driveSubsystem = driveSubsystem;

    addRequirements(m_shooterSubsystem, m_shooterPivotSubsystem, m_shooterIndexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterPivotSubsystem.setPosition(60);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_shooterPivotSubsystem.getPosition() == 60) {
      return true;
    }
    return false;
  }
}
