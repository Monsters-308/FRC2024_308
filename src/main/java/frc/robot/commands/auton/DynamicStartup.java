package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoAimDynamic;
import frc.robot.commands.shooterPivot.DynamicPivotToSpeaker;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;


public class DynamicStartup extends SequentialCommandGroup{
    
    /** 
     * Command for launching first note in auton.
     * Unlike static startup, this uses auto aim features so the robot can start anywhere on the field.
     */
    public DynamicStartup(ShooterSubsystem m_shooterSubsystem, ShooterIndexSubsystem m_shooterIndexSubsystem, LEDSubsystem m_LEDSubsystem,
                        ShooterPivotSubsystem m_shooterPivotSubsystem, VisionSubsystem m_visionSubsystem, DriveSubsystem m_driveSubsystem,
                        LEDSubsystem mLedSubsystem){ 
        addCommands(
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> m_shooterSubsystem.setPercent(1)),
                    new WaitCommand(1),
                    new AutonShootNote(m_shooterIndexSubsystem, m_LEDSubsystem)
                ),
                new DynamicPivotToSpeaker(m_shooterPivotSubsystem, m_driveSubsystem),
                new AutoAimDynamic(m_visionSubsystem, m_driveSubsystem, () -> 0, () -> 0, mLedSubsystem)
            )
        );
    }
}
