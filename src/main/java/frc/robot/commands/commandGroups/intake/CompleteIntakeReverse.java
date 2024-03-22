package frc.robot.commands.commandGroups.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterIndexConstants;
import frc.robot.commands.LED.setLED;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooterIndex.RunShooterIndexer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;

public class CompleteIntakeReverse extends SequentialCommandGroup  {

    /** Intakes a note from the ground and then puts it into the shooter index */
    public CompleteIntakeReverse(IntakeSubsystem intakeSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, LEDSubsystem LEDsubsystem){
        addCommands(
            new setLED(LEDsubsystem, LEDsubsystem::yellow),

            new ParallelCommandGroup(
                new RunIntake(intakeSubsystem, -IntakeConstants.kIntakeSpeed),
                new RunShooterIndexer(shooterIndexSubsystem, -ShooterIndexConstants.kIndexIntakeSpeed)

            ),

            new InstantCommand(() -> LEDsubsystem.setLEDFunction(LEDsubsystem::rainbow))
        );
    }

}
