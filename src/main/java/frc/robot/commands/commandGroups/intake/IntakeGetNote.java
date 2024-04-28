package frc.robot.commands.commandGroups.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.commands.LED.setLED;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intakePivot.IntakeGoStatic;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class IntakeGetNote extends SequentialCommandGroup  {

    /** First half of intaking sequence: bring intake down and run it until a note is detected. */
    public IntakeGetNote(IntakeSubsystem intakeSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, IntakePivotSubsystem intakePivotSubsystem, LEDSubsystem LEDsubsystem){
        addCommands(
            new setLED(LEDsubsystem, LEDsubsystem::red),

            // Get the shooter pivot started early
            new InstantCommand(() -> shooterPivotSubsystem.setPosition(ShooterPivotConstants.kShooterPivotDeckPosition), shooterPivotSubsystem),

            // Put intake down and run intake until a note is detected
            new ParallelCommandGroup(
                new IntakeGoStatic(intakePivotSubsystem, IntakePivotConstants.kIntakeDownPosition),
                new RunIntake(intakeSubsystem, 1)
                    .until(() -> intakeSubsystem.gamePieceDetected())
            ),

            new setLED(LEDsubsystem, LEDsubsystem::yellow)
        );
    }
}
