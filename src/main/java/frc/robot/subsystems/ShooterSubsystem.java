package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax m_topShooterMotor = new CANSparkMax(ShooterConstants.kTopShooterMotorCanID, MotorType.kBrushless);
  private final CANSparkMax m_bottomShooterMotor = new CANSparkMax(ShooterConstants.kBottomShooterMotorCanID, MotorType.kBrushless);

  private final DigitalInput m_noteSensor = new DigitalInput(ShooterConstants.kDigitalSensorPin);

  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  private final RelativeEncoder shooterMotorEncoderTop;
  private final RelativeEncoder shooterMotorEncoderBottom;
  private final SparkPIDController shooterMotorTopPIDController;
  private final SparkPIDController shooterMotorBottomPIDController;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    m_topShooterMotor.restoreFactoryDefaults(); 
    m_bottomShooterMotor.restoreFactoryDefaults(); 

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    shooterMotorEncoderTop = m_topShooterMotor.getEncoder();
    shooterMotorEncoderBottom = m_bottomShooterMotor.getEncoder();

    shooterMotorTopPIDController = m_topShooterMotor.getPIDController();
    shooterMotorBottomPIDController = m_bottomShooterMotor.getPIDController();

    shooterMotorTopPIDController.setFeedbackDevice(shooterMotorEncoderTop);
    shooterMotorBottomPIDController.setFeedbackDevice(shooterMotorEncoderBottom);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    //get the actual rpm of the spin motors, or get the estimated linear speed of the rollers like in the drive subystem
    shooterMotorEncoderTop.setVelocityConversionFactor(ShooterConstants.kShooterEncoderVelocityFactor);
    shooterMotorEncoderBottom.setVelocityConversionFactor(ShooterConstants.kShooterEncoderVelocityFactor);

    // Set the PID gains for the turning motor.
    shooterMotorTopPIDController.setP(ShooterConstants.kShooterP);
    shooterMotorTopPIDController.setI(ShooterConstants.kShooterI);
    shooterMotorTopPIDController.setD(ShooterConstants.kShooterD);
    shooterMotorTopPIDController.setFF(ShooterConstants.kShooterFF);
    shooterMotorTopPIDController.setOutputRange(ShooterConstants.kShooterMinOutput,
        ShooterConstants.kShooterMaxOutput);
    //bottom
    shooterMotorBottomPIDController.setP(ShooterConstants.kShooterP);
    shooterMotorBottomPIDController.setI(ShooterConstants.kShooterI);
    shooterMotorBottomPIDController.setD(ShooterConstants.kShooterD);
    shooterMotorBottomPIDController.setFF(ShooterConstants.kShooterFF);
    shooterMotorBottomPIDController.setOutputRange(ShooterConstants.kShooterMinOutput,
        ShooterConstants.kShooterMaxOutput);

    // Limit the current so the motor doesn't oof itself
    m_topShooterMotor.setSmartCurrentLimit(ShooterConstants.kShooterMotorCurrentLimit); 
    m_bottomShooterMotor.setSmartCurrentLimit(ShooterConstants.kShooterMotorCurrentLimit);
    
    //setting motors to spin freely (like ur mummy)
    m_topShooterMotor.setIdleMode(IdleMode.kCoast); 
    m_bottomShooterMotor.setIdleMode(IdleMode.kCoast);

    // Save the motor controller's configuration
    m_topShooterMotor.burnFlash(); 
    m_bottomShooterMotor.burnFlash();

    // Add Values to shuffleboard
    shooterTab.addBoolean("Game Piece", () -> gamePieceDetected());
    shooterTab.addDouble("Top Roller Speed", () -> getTopSpeed());
    shooterTab.addDouble("Bottom Roller Speed", () -> getBottomSpeed());

  }

  public void setShooterSpeed(double speed){
    shooterMotorBottomPIDController.setReference(speed, CANSparkBase.ControlType.kVelocity);
    shooterMotorTopPIDController.setReference(speed, CANSparkBase.ControlType.kVelocity);
  }

  public double getTopSpeed(){
    return shooterMotorEncoderTop.getVelocity();
  }

  public double getBottomSpeed(){
    return shooterMotorEncoderBottom.getVelocity();
  }

  /**
   * Returns whether or not there's a game piece in the intake.
   * pancake pancake pancake pancake pancake pancake pancake pancake
   * @return true if the sensor detects a game piece.
   */
  public boolean gamePieceDetected(){
    if (ShooterConstants.kSensorInverted){
      return !m_noteSensor.get();
    }

    return m_noteSensor.get();
  }  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
