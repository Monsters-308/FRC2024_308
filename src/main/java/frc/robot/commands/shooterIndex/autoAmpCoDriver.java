package frc.robot.commands.shooterIndex;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.shooter.autoWheelRevAndPivot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.AutoConstants;


class autoAmpCoDriver extends SequentialCommandGroup  {

    /** Moves the robot in front of the amp  */
    public autoAmpCoDriver(ShooterPivotSubsystem shooterPivotSubsystem, DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem){
        addCommands(
            new autoWheelRevAndPivot(shooterPivotSubsystem, driveSubsystem, shooterSubsystem, AutoConstants.kWheelSpeedAmp, AutoConstants.kAngleAmp)
        );
    }
}
