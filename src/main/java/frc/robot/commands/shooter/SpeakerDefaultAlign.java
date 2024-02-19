package frc.robot.commands.shooter;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem; 
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
  

    
public class SpeakerDefaultAlign extends Command {  
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;    
  private final ShooterIndexSubsystem m_shooterIndexSubsystem;

  //variables for motor speeds/velocities 



  public SpeakerDefaultAlign(ShooterSubsystem Subsystem, ShooterPivotSubsystem pivotSubsystem, ShooterIndexSubsystem indexSubsystem) {
    m_shooterSubsystem = Subsystem; 
    m_shooterPivotSubsystem = pivotSubsystem; 
    m_shooterIndexSubsystem = indexSubsystem;

    addRequirements(m_shooterSubsystem, m_shooterPivotSubsystem, m_shooterIndexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterPivotSubsystem.setPosition(45); //

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_shooterPivotSubsystem.getPosition() == 45){
        return true; 
    } 
    return false;
  }
}