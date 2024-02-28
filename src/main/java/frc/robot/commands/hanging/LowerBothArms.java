package frc.robot.commands.hanging;

import frc.robot.subsystems.HangingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HangingConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;

//skeleton made by nico
public class LowerBothArms extends Command {
  private final HangingSubsystem m_hangingSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LowerBothArms(HangingSubsystem hangingSubsystem) {
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
    m_hangingSubsystem.setBothSpeed(-0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hangingSubsystem.setBothSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hangingSubsystem.leftFullyRetracted() && m_hangingSubsystem.rightFullyRetracted();
  }
}
