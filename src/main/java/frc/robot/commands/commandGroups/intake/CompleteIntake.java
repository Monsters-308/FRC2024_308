package frc.robot.commands.commandGroups.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.IntakeDeck;

public class CompleteIntake extends SequentialCommandGroup  {

    /** Intakes a note from the ground and then puts it into the shooter index */
    public CompleteIntake(IntakeSubsystem intakeSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, IndexSubsystem indexSubsystem, IntakePivotSubsystem intakePivotSubsystem){
        new SequentialCommandGroup(
                new IntakeNote(intakeSubsystem, shooterPivotSubsystem, shooterIndexSubsystem, indexSubsystem, intakePivotSubsystem),
                new IntakeDeck(intakeSubsystem, shooterPivotSubsystem, shooterIndexSubsystem, indexSubsystem, intakePivotSubsystem)
            );
    }
}
