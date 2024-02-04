package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
    final private NetworkTableEntry ty;
    final private NetworkTableEntry tx; 
    final private NetworkTableEntry tv;
    final private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // Field to visualize april tag detection
    private final Field2d m_field = new Field2d();

    private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

    public VisionSubsystem(){
        ty = limelightTable.getEntry("ty");
        tx = limelightTable.getEntry("tx");
        tv = limelightTable.getEntry("tv");

        setPipeline(VisionConstants.kDefaultPipeline);

        visionTab.addInteger("Pipeline", () -> getPipeline());
        visionTab.add(m_field);
    }

    //tv = valid targets
    //tx horizontal offset from crosshair to target
    //ty vertical offset from crosshair to target
    //ta = target area 0% to 100%
    
    public void setPipeline(int pipeline){
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    public int getPipeline(){
        return ((Double)limelightTable.getEntry("pipeline").getNumber(-1)).intValue();
    }

    /**
     * Returns the horizontal offset from the crosshair to the target. Returns 0 if the target can't be found.
     */
    public double getX(){
        return tx.getDouble(0.0);
    }

    /**
     * Returns the vertical offset from the crosshair to the target. Returns 0 if the target can't be found.
     */
    public double getY(){
        return ty.getDouble(0.0);
    }
    
    /**
     * Returns the number of valid targets detected by the limelight. Returns 0 if no targets are found.
     */
    public int getTV(){
        return (int)tv.getDouble(0);
    }

    /**
     * Uses whatever april tag is in front of it to estimate the robot's position on the field. 
     * Returns null if no april tag is in view.
     * @return The position of the robot, or null.
     */
    public Pose2d getRobotPosition(){
        
        if(getPipeline() == VisionConstants.kAprilTagPipeline) {
            /* Notes for Gabe: a Pose2d object contains a robot's x position, y position, 
             * and rotation. I need your help with taking the limelight values from network tables 
             * and converting them to a robot pose object. Once you do that I'll figure out how
             * to use this object for correcting the odometry.
             */
            double [] robotPosArrary = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
            SmartDashboard.putNumber("robotposarr0", robotPosArrary[0]);
            return new Pose2d(robotPosArrary[0], robotPosArrary[1], new Rotation2d(robotPosArrary[2]));
        }
        return null;
    }


    @Override
    public void periodic(){
        Pose2d limelightPose = getRobotPosition();
        if(limelightPose != null){
            m_field.setRobotPose(limelightPose);
        }
    }
}