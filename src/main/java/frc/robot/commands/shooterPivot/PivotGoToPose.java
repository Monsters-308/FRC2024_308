package frc.robot.commands.shooterPivot;

import frc.robot.subsystems.ShooterPivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
  //:3

public class PivotGoToPose extends Command {  
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;  
  private final double m_pivotAngle;
  
  /**
   * Sets the shooter pivot to a certain angle.
   * NOTE: this command is equivalent to new InstantCommand(() -> m_shooterPivotSubsystem.setPosition(pivotAngle))
   * except the command only ends once the pivot has reached its desired angle.
   * @param pivotSubsystem
   * @param pivotAngle
   */
  public PivotGoToPose(ShooterPivotSubsystem pivotSubsystem, double pivotAngle) {
    m_shooterPivotSubsystem = pivotSubsystem; 
    m_pivotAngle = pivotAngle; 

    addRequirements(m_shooterPivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterPivotSubsystem.setPosition(m_pivotAngle); 

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooterPivotSubsystem.inPosition();
  }
}