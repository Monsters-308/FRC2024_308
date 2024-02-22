package frc.robot.commands.intake;

import frc.robot.commands.shooter.LaunchNote;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
    
public class QuickIntake extends ParallelRaceGroup {

  public QuickIntake(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem, ShooterIndexSubsystem shooterIndexSubsystem) {
    deadlineWith(new LaunchNote(shooterIndexSubsystem));
  }
}
