package frc.robot.commands.commandGroups.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.commands.LED.setLED;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intakePivot.IntakeGoDynamic;
import frc.robot.commands.intakePivot.IntakeGoStatic;
import frc.robot.commands.shooterIndex.IndexNote;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class IntakeIndexNote extends SequentialCommandGroup  {

    /** Second half of intaking sequence: launch note from intake into shooter indexer. */
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
