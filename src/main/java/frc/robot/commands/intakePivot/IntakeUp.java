package frc.robot.commands.intakePivot;

import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
    
public class IntakeUp extends Command {
  private final ShooterPivotSubsystem m_shooterPivotSubsystem; 
  private final IntakePivotSubsystem m_intakePivotSubsystem;
  //variables for motor speeds/velocities


  public IntakeUp(ShooterPivotSubsystem shooterPivotSubsystem, IntakePivotSubsystem intakePivotSubsystem) {
    m_shooterPivotSubsystem = shooterPivotSubsystem; 
    m_intakePivotSubsystem = intakePivotSubsystem;
    addRequirements(intakePivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterPivotSubsystem.setPosition(ShooterPivotConstants.kshooterPivotUpPosition);
    //m_intakePivotSubsystem.intakeUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //
    return false;
  }
}
