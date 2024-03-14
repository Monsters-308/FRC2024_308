package frc.robot.commands.hanging;

import frc.robot.subsystems.HangingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


//skeleton made by nico
public class RaiseBothArms extends Command {
  private final HangingSubsystem m_hangingSubsystem;

  /**
   * Raises both hanging arms until they are both fully extended.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RaiseBothArms(HangingSubsystem hangingSubsystem) {
    m_hangingSubsystem = hangingSubsystem;
    
    addRequirements(m_hangingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hangingSubsystem.setBothSpeed(0.8);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hangingSubsystem.setBothSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hangingSubsystem.leftFullyExtended() && m_hangingSubsystem.rightFullyExtended();
  }
}
