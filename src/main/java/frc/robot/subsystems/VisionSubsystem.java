package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.vision.UpdateOdometry;
import frc.utils.FieldUtils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class VisionSubsystem extends SubsystemBase {
    final private NetworkTableEntry ty;
    final private NetworkTableEntry tx; 
    final private NetworkTableEntry tv;
    final private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // Field to visualize april tag detection
    private final Field2d m_field = new Field2d();
    private boolean setOdometry = false; 
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

    public double getTV(){
        return tv.getDouble(0.0);
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
    public int getTargets(){
        return (int)tv.getDouble(0);
    }

    public double getTimeStamp(){
        // Check if the limelight is actually viewing an apriltag
        if((getPipeline() == VisionConstants.kAprilTagPipeline) && (getTargets() > 0)) {

            // Get pose data from networktables
            double[] robotPosArrary = limelightTable.getEntry(
                FieldUtils.isRedAlliance() ?
                    "botpose_wpired" :
                    "botpose_wpiblue"
            ).getDoubleArray(new double[6]);

            // Turn pose data into pose2d object
            return (robotPosArrary[6]);
        }
        return 0;
    }

    public double getLastTimeStamp(){
        // Check if the limelight is actually viewing an apriltag
        if((getPipeline() == VisionConstants.kAprilTagPipeline) && (getTargets() > 0)) {

            // Get pose data from networktables
            double robotLastTimeStamp = limelightTable.getEntry(
                FieldUtils.isRedAlliance() ?
                    "botpose_wpired" :
                    "botpose_wpiblue"
            ).getLastChange();

            // Turn pose data into pose2d object
            return (robotLastTimeStamp);
        }
        return 0;
    }

    /**
     * Uses whatever april tag is in front of the limelight to estimate the robot's position on the field. 
     * Returns null if no april tag is in view.
     * @return The position of the robot according to apriltags as a Pose2d object, or null. 
     */
    public Pose2d getRobotPosition(){
        // Check if the limelight is actually viewing an apriltag
        if((getPipeline() == VisionConstants.kAprilTagPipeline) && (getTargets() > 0)) {

            // Get pose data from networktables
            double[] robotPosArrary = limelightTable.getEntry(
                FieldUtils.isRedAlliance() ?
                    "botpose_wpired" :
                    "botpose_wpiblue"
            ).getDoubleArray(new double[6]);

            // Turn pose data into pose2d object
            return new Pose2d(robotPosArrary[0], robotPosArrary[1], Rotation2d.fromDegrees(robotPosArrary[5]));
        }
        return null;
    }


    @Override
    public void periodic(){
        // Display april tag data on a field widget for testing
        Pose2d limelightPose = getRobotPosition();
        if(limelightPose != null){
            m_field.setRobotPose(FieldUtils.redWidgetFlip(limelightPose));
        }        
    }

}