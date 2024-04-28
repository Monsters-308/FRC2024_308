package frc.robot.commands.commandGroups.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class CompleteIntake extends SequentialCommandGroup  {

    /** Intakes a note from the ground and then puts it into the shooter index */
    public CompleteIntake(IntakeSubsystem intakeSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, IntakePivotSubsystem intakePivotSubsystem, LEDSubsystem LEDsubsystem){
        addCommands(
            new IntakeGetNote(intakeSubsystem,shooterPivotSubsystem,shooterIndexSubsystem, intakePivotSubsystem, LEDsubsystem),
            new IntakeIndexNote(intakeSubsystem,shooterPivotSubsystem,shooterIndexSubsystem, intakePivotSubsystem, LEDsubsystem)
        );
    }
}
