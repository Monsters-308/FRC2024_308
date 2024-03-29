package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class CoolRevWheels extends Command {

  private final ShooterSubsystem m_shooterSubsystem;
  private final XboxController m_coDriverController;

  /**
   * Revs the shooter wheels and vibrates the coDriver controller when the rollers are up to speed.
   */
  public CoolRevWheels(ShooterSubsystem shooterSubsystem, XboxController coDriverController) {
    
    m_shooterSubsystem = shooterSubsystem;
    m_coDriverController = coDriverController;

    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setPercent(1);

    // Rumble CoDriver controller if shooter wheels are fast enough
    if((m_shooterSubsystem.getTopSpeed() > 28) && (m_shooterSubsystem.getBottomSpeed() > 28)){
      m_coDriverController.setRumble(RumbleType.kBothRumble, 0.5);
    }
    else{
      m_coDriverController.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopRollers();
    m_coDriverController.setRumble(RumbleType.kBothRumble, 0);
  }
}
