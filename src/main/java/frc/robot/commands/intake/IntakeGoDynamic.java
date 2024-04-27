package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;

public class IntakeGoDynamic extends Command {
  private final IntakePivotSubsystem m_intakePivotSubsystem;
  private final double m_position;

  /**
   * Runs the intake rollers at a specified speed.
   * NOTE: this is equivalent to new InstantCommand(() -> m_intakeSubsystem.setSpeed(speed), m_intakeSubsystem) 
   * except it stops the motor when interrupted.
   * @param intakeSubsystem
   */
  public IntakeGoDynamic(IntakePivotSubsystem intakeSubsystem, double position) {
    m_intakePivotSubsystem = intakeSubsystem;
    m_position = position;

    addRequirements(m_intakePivotSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_intakePivotSubsystem.setPosition(m_position);
  }

  @Override
  public boolean isFinished() {
    return m_intakePivotSubsystem.inPosition();
  }

}
