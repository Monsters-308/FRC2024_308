package frc.robot.commands.commandGroups.drive;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.RobotGotoFieldPos;
import frc.robot.commands.shooter.autoWheelRevAndPivot;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class autoAmp extends ParallelCommandGroup  {

    /** Moves the robot in front of the amp */
    public autoAmp(DriveSubsystem driveSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterSubsystem shooterSubsystem){
        addCommands(
            new ParallelCommandGroup(
                new RobotGotoFieldPos(driveSubsystem, FieldConstants.kAmpScoringPosition, true),
                new autoWheelRevAndPivot(shooterPivotSubsystem, driveSubsystem, shooterSubsystem, AutoConstants.kWheelSpeedAmp, AutoConstants.kAngleAmp)
            )
        );
    }
}
