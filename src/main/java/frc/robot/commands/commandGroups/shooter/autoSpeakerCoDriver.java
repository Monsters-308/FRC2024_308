package frc.robot.commands.shooter;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.AutoConstants;


public class autoSpeakerCoDriver extends SequentialCommandGroup  {

    /** Moves the robot in front of the amp  */
    public autoSpeakerCoDriver(ShooterPivotSubsystem shooterPivotSubsystem, DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem){
        addCommands(
            new ParallelCommandGroup(
                new autoWheelRevAndPivot(shooterPivotSubsystem, driveSubsystem, shooterSubsystem, AutoConstants.kWheelSpeedSpeaker, AutoConstants.kAngleSpeaker)
            )
        );
    }
}
