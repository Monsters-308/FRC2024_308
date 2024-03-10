package frc.robot.commands.commandGroups.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.commands.LED.setLED;
import frc.robot.commands.intake.IntakeDeck;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intakePivot.SetIntakeAngle;
import frc.robot.commands.shooterIndex.IndexNoteGood;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class CompleteIntake extends SequentialCommandGroup  {

    /** Intakes a note from the ground and then puts it into the shooter index */
    public CompleteIntake(IntakeSubsystem intakeSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, IndexSubsystem indexSubsystem, IntakePivotSubsystem intakePivotSubsystem, LEDSubsystem LEDsubsystem){
        addCommands(
            new setLED(LEDsubsystem, LEDsubsystem::red),

            // Get the shooter pivot started early
            new InstantCommand(() -> shooterPivotSubsystem.setPosition(ShooterPivotConstants.kshooterPivotDeckPosition)),
            new ParallelCommandGroup(
                new SetIntakeAngle(intakePivotSubsystem, IntakePivotConstants.kIntakeDownPosition),
                new RunIntake(intakeSubsystem, 1)
                    .until(() -> intakeSubsystem.gamePieceDetected())
            ),
            new setLED(LEDsubsystem, LEDsubsystem::yellow),
            new SetIntakeAngle(intakePivotSubsystem, IntakePivotConstants.kIntakeDeckPosition),

            new WaitUntilCommand(() -> shooterPivotSubsystem.inPosition()),

            new ParallelDeadlineGroup(
                new IndexNoteGood(shooterIndexSubsystem), 
                new IntakeDeck(intakeSubsystem, indexSubsystem) 
            ),
            new InstantCommand(() -> shooterPivotSubsystem.setPosition(ShooterPivotConstants.kShooterPivotPoduim), shooterPivotSubsystem),

            // Add an InstantCommand to reset the LED state after the command group finishes
            new InstantCommand(() -> LEDsubsystem.setLEDFunction(LEDsubsystem::rainbow))
        );
    }

}
