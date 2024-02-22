package frc.robot.commands.commandGroups.intake;

import frc.robot.commands.commandGroups.shooter.LaunchNote;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
    
public class QuickIntake extends ParallelRaceGroup {

  public QuickIntake(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem, ShooterIndexSubsystem shooterIndexSubsystem) {
    deadlineWith(new LaunchNote(shooterIndexSubsystem));
  }
}
