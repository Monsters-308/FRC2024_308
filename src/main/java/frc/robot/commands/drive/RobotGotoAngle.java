package frc.robot.commands.drive;
/*
 * The template code does not have a function for setting the robot to face a certain way, so we're just gonna have to implement that
 * ourselves using the drive function, the getHeading function, and a pid controller that we'll have to tune.
 */

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.HeadingConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotGotoAngle extends Command {

    private final DriveSubsystem m_driveSubsystem;

    private final PIDController pidController = new PIDController(HeadingConstants.kHeadingP, 
                                                                  HeadingConstants.kHeadingI, 
                                                                  HeadingConstants.kHeadingD);
    private boolean m_complete = false;

    private final double m_desiredAngle;

    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;

    /** 
     * Uses PID to make the robot rotate to a certain direction while still giving the driver control over the translation of the robot.
     */
    public RobotGotoAngle(DriveSubsystem subsystem, double angle, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        m_driveSubsystem = subsystem;

        m_desiredAngle = angle;

        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;

        pidController.enableContinuousInput(-180, 180);

        pidController.setTolerance(HeadingConstants.kHeadingTolerance);

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
        pidController.reset();
        pidController.setSetpoint(m_desiredAngle);

    }

    /*
     * This function is called repeatedly when the schedueler's "run()" function is
     * called.
     * Once you want the function to end, you should set m_complete to true.
     */
    // When not overridden, this function is blank.
    @Override
    public void execute() {

        double rotation = pidController.calculate(m_driveSubsystem.getHeading());

        rotation = MathUtil.clamp(rotation, HeadingConstants.kHeadingMinOutput, HeadingConstants.kHeadingMaxOutput);

        m_driveSubsystem.drive(
            -MathUtil.applyDeadband(m_xSpeed.getAsDouble(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_ySpeed.getAsDouble(), OIConstants.kDriveDeadband),
            rotation,
            true, false
        );
        
        if(pidController.atSetpoint()){
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
