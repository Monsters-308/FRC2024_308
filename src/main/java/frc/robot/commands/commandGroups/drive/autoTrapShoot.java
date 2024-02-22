package frc.robot.commands.commandGroups.drive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.RobotGotoFieldPos;
import frc.robot.commands.shooter.autoWheelRevAndPivot;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class autoTrapShoot extends ParallelCommandGroup  {

    public autoTrapShoot(DriveSubsystem driveSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterSubsystem shooterSubsystem){
        addCommands(
            new ParallelCommandGroup(
                new RobotGotoFieldPos(driveSubsystem, 40, 10, 90), // x y and z of the postion the robot should be in to score the amp, depends on alliance??
                new autoWheelRevAndPivot(shooterPivotSubsystem, driveSubsystem, shooterSubsystem, AutoConstants.kWheelSpeedAmp, AutoConstants.kAngleAmp)
            )
        );
    }
}