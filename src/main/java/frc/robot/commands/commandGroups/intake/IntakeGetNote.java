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

public class IntakeGetNote extends SequentialCommandGroup  {

    /** Intakes a note from the ground and then puts it into the shooter index */
    public IntakeGetNote(IntakeSubsystem intakeSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, IntakePivotSubsystem intakePivotSubsystem, LEDSubsystem LEDsubsystem){
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
