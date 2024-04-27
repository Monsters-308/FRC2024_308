package frc.robot.commands.commandGroups.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.commands.LED.setLED;
import frc.robot.commands.intake.IntakeGoDynamic;
import frc.robot.commands.intake.IntakeGoStatic;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intakePivot.SetIntakeAngle;
import frc.robot.commands.shooterIndex.IndexNote;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class IntakeIndexNote extends SequentialCommandGroup  {

    /** Intakes a note from the ground and then puts it into the shooter index */
    public IntakeIndexNote(IntakeSubsystem intakeSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, IntakePivotSubsystem intakePivotSubsystem, LEDSubsystem LEDsubsystem){
        addCommands(
            new setLED(LEDsubsystem, LEDsubsystem::yellow),
            // Bring intake up
            new IntakeGoDynamic(intakePivotSubsystem, IntakePivotConstants.kIntakeDeckPosition),

            // Make sure shooter is fully pivoted up
            new WaitUntilCommand(() -> shooterPivotSubsystem.inPosition()),

            // Put note in indexer
            new ParallelDeadlineGroup(
                new IndexNote(shooterIndexSubsystem), 
                new RunIntake(intakeSubsystem, IntakeConstants.kIntakeSpeed) 
            ),
            new IntakeGoStatic(intakePivotSubsystem, IntakePivotConstants.kIntakeDeckPosition),

            // Automatically start putting shooter up again
            //new InstantCommand(() -> shooterPivotSubsystem.setPosition(ShooterPivotConstants.kShooterPivotSpeakerPosition), shooterPivotSubsystem),

            // Add an InstantCommand to reset the LED state after the command group finishes
            new InstantCommand(() -> LEDsubsystem.setLEDFunction(LEDsubsystem::rainbow))
        );
    }
}
