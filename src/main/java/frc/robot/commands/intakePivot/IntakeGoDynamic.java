package frc.robot.commands.intakePivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;

public class IntakeGoDynamic extends Command {
  private final IntakePivotSubsystem m_intakePivotSubsystem;
  private final double m_position;

  /** 
   * Sets the intake pivot to a specific angle using PID control.
   * The command the same as IntakeGoStatic except it leave PID control enabled.
   */
  public IntakeGoDynamic(IntakePivotSubsystem intakeSubsystem, double position) {
    m_intakePivotSubsystem = intakeSubsystem;
    m_position = position;

    addRequirements(m_intakePivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_intakePivotSubsystem.setPosition(m_position);
  }

  @Override
  public boolean isFinished() {
    return m_intakePivotSubsystem.inPosition();
  }

}
