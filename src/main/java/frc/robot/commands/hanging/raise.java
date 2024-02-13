package frc.robot.commands.hanging;

import frc.robot.subsystems.HangingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class raise extends Command { 
    
  private final HangingSubsystem m_hangingSubsystem;

  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public raise(HangingSubsystem hangingSubsystem) {
    m_hangingSubsystem = hangingSubsystem;
    
    addRequirements(m_hangingSubsystem);
  }

  // Called when the command is initially scheduled. 
  @Override
  public void initialize() {
    if 
    (!m_hangingSubsystem.leftFullyExtended()){
 m_hangingSubsystem.setLeftSpeed(0.8);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
