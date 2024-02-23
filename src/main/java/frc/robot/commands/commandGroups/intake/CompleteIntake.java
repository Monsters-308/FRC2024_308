package frc.robot.commands.commandGroups.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.shooterIndex.IndexNote;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.commands.intake.IntakeDeck;

public class CompleteIntake extends SequentialCommandGroup  {

    /** Intakes a note from the ground and then puts it into the shooter index */
    public CompleteIntake(IntakeSubsystem intakeSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, IndexSubsystem indexSubsystem, IntakePivotSubsystem intakePivotSubsystem){
        addCommands(
            // Get the shooter pivot started early
            new InstantCommand(() -> shooterPivotSubsystem.setPosition(ShooterPivotConstants.kshooterPivotDeckPosition)),
            new IntakeNote(intakeSubsystem, intakePivotSubsystem),

            new ParallelDeadlineGroup(
                new IndexNote(shooterIndexSubsystem), 
                new IntakeDeck(intakeSubsystem, shooterPivotSubsystem, indexSubsystem, intakePivotSubsystem)
            )
        );
    }
}
