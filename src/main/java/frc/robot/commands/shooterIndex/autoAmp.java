package frc.robot.commands.shooterIndex;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.drive.RobotGotoFieldPos;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.utils.FieldUtils;

class autoAmp extends SequentialCommandGroup  {

    /** Moves the robot in front of the amp  */
    public autoAmp(DriveSubsystem driveSubsystem){
        addCommands(
            new RobotGotoFieldPos(driveSubsystem, 40, FieldUtils.flipRedY(69), FieldUtils.flipRedAngle(-90))
        );
    }
}
