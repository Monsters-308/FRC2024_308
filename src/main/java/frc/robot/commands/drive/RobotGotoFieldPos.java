package frc.robot.commands.drive;
/*
 * The template code does not have a function for setting the robot to face a certain way, so we're just gonna have to implement that
 * ourselves using the drive function, the getHeading function, and a pid controller that we'll have to tune.
 */

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.OdometryUtils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.HeadingConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotGotoFieldPos extends Command {

    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;

    private final PIDController pidControllerX = new PIDController(HeadingConstants.kHeadingP, 
                                                                  HeadingConstants.kHeadingI, 
                                                                  HeadingConstants.kHeadingD);
    private final PIDController pidControllerY = new PIDController(HeadingConstants.kHeadingP, 
                                                                  HeadingConstants.kHeadingI, 
                                                                  HeadingConstants.kHeadingD);
    private final PIDController pidControllerZ = new PIDController(HeadingConstants.kHeadingP, 
                                                                  HeadingConstants.kHeadingI, 
                                                                  HeadingConstants.kHeadingD);
    private boolean m_complete = false;

    private final Pose2d m_desiredRobotoPos;

    /** 
     * Uses PID to make the robot go to a certain postion relative to the field.  
     */
    public RobotGotoFieldPos(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Pose2d desiredRobotoPos) {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;

        m_desiredRobotoPos = desiredRobotoPos;

        pidControllerX.setTolerance(HeadingConstants.kHeadingTolerance);
        pidControllerY.setTolerance(HeadingConstants.kHeadingTolerance);
        pidControllerZ.setTolerance(HeadingConstants.kHeadingTolerance);

        addRequirements(m_driveSubsystem);
    }

    /*
     * This function is called once when the command is schedueled.
     * If you are overriding "isFinished()", you should probably use this to set
     * m_complete to false so the command doesn't
     * instantly end.
     */
    // When not overridden, this function is blank.
    @Override
    public void initialize() {
        m_complete = false;
        pidControllerX.reset();
        pidControllerY.reset();
        pidControllerZ.reset();

        pidControllerX.setSetpoint(m_desiredRobotoPos.getX());
        pidControllerY.setSetpoint(m_desiredRobotoPos.getX());
        pidControllerZ.setSetpoint(m_driveSubsystem.getHeading());

    }

    /*
     * This function is called repeatedly when the schedueler's "run()" function is
     * called.
     * Once you want the function to end, you should set m_complete to true.
     */
    // When not overridden, this function is blank.
    @Override
    public void execute() {
        Pose2d pos = m_visionSubsystem.getRobotPosition();

        double x = pidControllerX.calculate(pos.getTranslation().getX());
        double y = pidControllerY.calculate(pos.getTranslation().getY());
        double z = pidControllerZ.calculate(m_driveSubsystem.getHeading());

        x = MathUtil.clamp(x, HeadingConstants.kHeadingMinOutput, HeadingConstants.kHeadingMaxOutput);
        y = MathUtil.clamp(y, HeadingConstants.kHeadingMinOutput, HeadingConstants.kHeadingMaxOutput);
        z = MathUtil.clamp(z, HeadingConstants.kHeadingMinOutput, HeadingConstants.kHeadingMaxOutput);


        m_driveSubsystem.drive(
            x,
            y,
            z,
            true, false
        );
        
        if(pidControllerX.atSetpoint() && pidControllerY.atSetpoint() && pidControllerZ.atSetpoint()){
            m_complete = true;
        }
    }


    /*
     * This function is called once when the command ends.
     * A command ends either when you tell it to end with the "isFinished()"
     * function below, or when it is interupted.
     * Whether a command is interrupted or not is determined by
     * "boolean interrupted."
     * Things initialized in "initialize()" should be closed here.
     */
    // When not overridden, this function is blank.
    @Override
    public void end(boolean interrupted) {

    }

    /*
     * This fuction is used to tell the robot when the command has ended.
     * This function is called after each time the "execute()" function is ran.
     * Once this function returns true, "end(boolean interrupted)" is ran and the
     * command ends.
     * It is recommended that you don't use this for commands that should run
     * continuously, such as drive commands.
     */
    // When not overridden, this function returns false.
    @Override
    public boolean isFinished() {
        return m_complete;
    }

}
