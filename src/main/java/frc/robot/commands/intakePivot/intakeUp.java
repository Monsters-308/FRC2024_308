package frc.robot.commands.intakePivot;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.Constants.ShooterPivotConstants; 
    
public class intakeUp extends Command {
  private final IntakePivotSubsystem m_intakePivotSubsystem;
  //variables for motor speeds/velocities


  public intakeUp(IntakePivotSubsystem intakePivotSubsystem) {
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
    m_intakePivotSubsystem.intakeUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_intakePivotSubsystem.isIntakeUp()){
      return true;
    }
    return false;
  }
}
