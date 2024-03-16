package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class checkRPMShake extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  double wheelSpeed = 0;
  double angle = 0;

  // variables for motor speeds/velocities

      /**

     */
  public checkRPMShake(ShooterSubsystem shooterSubsystem, double WheelSpeed) {
    m_shooterSubsystem = shooterSubsystem;
    wheelSpeed = WheelSpeed;


    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }



  @Override
  public boolean isFinished() {
    if ((m_shooterSubsystem.getTopSpeed() > wheelSpeed) && (m_shooterSubsystem.getBottomSpeed() > wheelSpeed)){
      return true;
    }
    return false;
  }
}
