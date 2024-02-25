package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final double m_speed;

  /**
   * Runs the intake rollers at a specified speed.
   * NOTE: this is equivalent to new InstantCommand(() -> m_intakeSubsystem.setSpeed(speed), m_intakeSubsystem) 
   * except it stops the motor when interrupted.
   * @param intakeSubsystem
   */
  public RunIntake(IntakeSubsystem intakeSubsystem, double speed) {
    m_intakeSubsystem = intakeSubsystem;
    m_speed = speed;

    addRequirements(m_intakeSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setSpeed(0);
  }

}
