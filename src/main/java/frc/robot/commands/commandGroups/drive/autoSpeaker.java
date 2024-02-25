package frc.robot.commands.commandGroups.drive;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.drive.RobotGotoFieldPos;
import frc.robot.commands.shooter.autoWheelRevAndPivot;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class autoSpeaker extends ParallelCommandGroup  {

    /** Moves robot in front of speaker */
    public autoSpeaker(DriveSubsystem driveSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterSubsystem shooterSubsystem){
        addCommands(
            new ParallelCommandGroup(
                new RobotGotoFieldPos(driveSubsystem, FieldConstants.kSpeakerScoringPosition), // x y and z of the postion the robot should be in to score the amp, depends on alliance??
                new autoWheelRevAndPivot(shooterPivotSubsystem, driveSubsystem, shooterSubsystem, AutoConstants.kWheelSpeedSpeaker, AutoConstants.kAngleSpeaker)
            )
        );
    }
}
