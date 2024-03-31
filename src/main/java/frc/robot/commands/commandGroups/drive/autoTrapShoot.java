package frc.robot.commands.commandGroups.drive;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.drive.RobotGotoFieldPos;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class autoTrapShoot extends ParallelCommandGroup  {
    
    /** Moves robot in front of trap */
    public autoTrapShoot(DriveSubsystem driveSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterSubsystem shooterSubsystem){
        addCommands(
            new ParallelCommandGroup(
                new RobotGotoFieldPos(driveSubsystem, FieldConstants.kTrapPositionAmpSide, true)
                //new autoWheelRevAndPivot(shooterPivotSubsystem, driveSubsystem, shooterSubsystem, AutoConstants.kWheelSpeedAmp, AutoConstants.kAngleAmp)
            )
        );
    }
}
