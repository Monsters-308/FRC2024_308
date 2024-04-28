package frc.robot.commands.intakePivot;

import frc.robot.Constants.IntakePivotConstants;
import frc.robot.subsystems.IntakePivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
    
public class SetIntakeAngle extends Command {
  private final IntakePivotSubsystem m_intakePivotSubsystem;
  private final double m_desiredAngle;

  /**
   * (Defunct) Moves the intake pivot to a specific angle. 
   * This command ends once the arm has reached its desired angle.
   * @param intakePivotSubsystem
   */
  public SetIntakeAngle(IntakePivotSubsystem intakePivotSubsystem, double angle) {
    m_intakePivotSubsystem = intakePivotSubsystem;
    m_desiredAngle = angle;

    addRequirements(intakePivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_desiredAngle - m_intakePivotSubsystem.getPosition().getDegrees() < 0){
      m_intakePivotSubsystem.setSpeed(IntakePivotConstants.kPivotSpeed);
    }
    else if (m_desiredAngle - m_intakePivotSubsystem.getPosition().getDegrees() > 0){
      m_intakePivotSubsystem.setSpeed(-IntakePivotConstants.kPivotSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakePivotSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_desiredAngle - m_intakePivotSubsystem.getPosition().getDegrees()) < IntakePivotConstants.kAngleTolerance;
  }
}
