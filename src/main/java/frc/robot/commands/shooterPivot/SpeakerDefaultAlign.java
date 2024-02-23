package frc.robot.commands.shooterPivot;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.subsystems.ShooterPivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
  

    
public class SpeakerDefaultAlign extends Command {  
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;  

  public SpeakerDefaultAlign(ShooterPivotSubsystem pivotSubsystem) {
    m_shooterPivotSubsystem = pivotSubsystem; 

    addRequirements(m_shooterPivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterPivotSubsystem.setPosition(ShooterPivotConstants.kShooterPivotSpeakerPosition); //

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_shooterPivotSubsystem.getPosition() == ShooterPivotConstants.kShooterPivotSpeakerPosition){
        return true; 
    } 
    return false;
  }
}