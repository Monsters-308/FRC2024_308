package frc.robot.commands.commandGroups.intake;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.utils.FieldUtils;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.shooter.autoWheelRevAndPivot;
import frc.robot.commands.drive.RobotGotoFieldPos;
import frc.robot.commands.intake.IntakeDeck;


public class CompleteIntake extends SequentialCommandGroup  {

    /** Moves the robot in front of the amp  */
    public CompleteIntake(IntakeSubsystem intakeSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, IndexSubsystem indexSubsystem, IntakePivotSubsystem intakePivotSubsystem){
        new SequentialCommandGroup(
                new IntakeNote(intakeSubsystem, shooterPivotSubsystem, shooterIndexSubsystem, indexSubsystem, intakePivotSubsystem),
                new IntakeDeck(intakeSubsystem, shooterPivotSubsystem, shooterIndexSubsystem, indexSubsystem, intakePivotSubsystem)
            );
    }
}
