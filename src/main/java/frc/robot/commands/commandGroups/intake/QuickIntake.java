package frc.robot.commands.commandGroups.intake;

import frc.robot.commands.shooterIndex.IndexNote;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
    
public class QuickIntake extends ParallelCommandGroup {

  /** Testing: simply runs all of the intake motors until a note is in the index subsystem */
  public QuickIntake(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem, ShooterIndexSubsystem shooterIndexSubsystem) {
    addCommands(
      new InstantCommand(() -> intakeSubsystem.setSpeed(1)),
      new InstantCommand(() -> indexSubsystem.setSpeed(1)),
      new IndexNote(shooterIndexSubsystem)
    );

    // Ending procedures:
    andThen(
      new InstantCommand(() -> intakeSubsystem.setSpeed(0)),
      new InstantCommand(() -> indexSubsystem.setSpeed(0))
    );
  }
}
