package frc.robot.commands.intakePivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;

public class IntakeGoStatic extends Command {
  private final IntakePivotSubsystem m_intakePivotSubsystem;
  private final double m_position;

  /** 
   * Sets the intake pivot to a specific angle using PID control.
   * Once the pivot is in position, it disables PID control and the command ends.
   */
  public IntakeGoStatic(IntakePivotSubsystem intakeSubsystem, double position) {
    m_intakePivotSubsystem = intakeSubsystem;
    m_position = position;

    addRequirements(m_intakePivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_intakePivotSubsystem.setPosition(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakePivotSubsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return m_intakePivotSubsystem.inPosition();
    
  }

}
