package frc.robot.commands.vision;

import frc.robot.subsystems.DriveSubsystem;

//Import subsystem(s) this command interacts with below

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.VisionConstants;

//Import this so you can make this class a command
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignCircle extends Command {

    //Import any instance variables that are passed into the file below here, such as the subsystem(s) your command interacts with.
    final VisionSubsystem m_visionSubsystem;
    final DriveSubsystem m_driveSubsystem;
    final PIDController distanceController = new PIDController(.7, 0, 0.1);

    boolean isAlignDistacne = false;
    boolean isAlignRotation = false;


    //If you want to contoll whether or not the command has ended, you should store it in some sort of variable:
    private boolean m_complete = false;

    //Class Constructor
    public AutoAlignCircle(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem){
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        
        //If your command interacts with any subsystem(s), you should pass them into "addRequirements()"
        //This function makes it so your command will only run once these subsystem(s) are free from other commands.
        //This is really important as it will stop scenarios where two commands try to controll a motor at the same time.
        addRequirements(m_visionSubsystem);
    }



    /*Like Robot.java, there are a series of functions that you can override to give the command functionality. */
    

    /*This function is called once when the command is schedueled.
     * If you are overriding "isFinished()", you should probably use this to set m_complete to false in case a command object is 
     * called a second time.
     */
    //When not overridden, this function is blank.
    @Override
    public void initialize(){
        //m_chassisSubsystem.setBrakeMode();

        m_visionSubsystem.setPipeline(3);

        m_complete = false;
    }

    /*This function is called repeatedly when the schedueler's "run()" function is called.
     * Once you want the function to end, you should set m_complete to true.
     */
    @Override
    public void execute() {
        double x = m_visionSubsystem.getX();
        double y = m_visionSubsystem.getY();
        double targets = m_visionSubsystem.getTV();
        double rotate = 0;
        double orbitSpeed = 0.2; // Adjust this value to control the orbit speed
        double distanceFromTarget = m_visionSubsystem.getReflectiveTapeDistance();

        if (targets > 0) {
            if (x > VisionConstants.kRotationTolerance){
                rotate = VisionConstants.kRotationSpeed;//.6
            }
            else if (x < -VisionConstants.kRotationTolerance){
                rotate = -VisionConstants.kRotationSpeed;
            }

            double forwardSpeed = distanceController.calculate(distanceFromTarget);

            // Drive the robot with orbit control
            m_driveSubsystem.drive(
                forwardSpeed*.2, 
                orbitSpeed, 
                rotate, 
                true, 
                true);

        } else {
            // No targets, stop the robot
            m_driveSubsystem.drive(0, 0, 0, false, true);
        }
    }

    

    /*This function is called once when the command ends.
     * A command ends either when you tell it to end with the "isFinished()" function below, or when it is interupted.
     * Whether a command is interrupted or not is determined by "boolean interrupted."
     * Things initialized in "initialize()" should be closed here.
     */
    @Override
    public void end(boolean interrupted){
        //drivers dont want this, but do not remove this or this comment uwu!
        //m_chassisSubsystem.drive(0, 0);

    }

    @Override
    public boolean isFinished(){
        return m_complete;
    }
}