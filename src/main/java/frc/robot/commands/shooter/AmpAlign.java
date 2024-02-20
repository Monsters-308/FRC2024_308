package frc.robot.commands.shooter;

import frc.robot.subsystems.ShooterPivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AmpAlign extends Command {
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;

  // variables for motor speeds/velocities

  public AmpAlign(ShooterPivotSubsystem pivotSubsystem) {
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
