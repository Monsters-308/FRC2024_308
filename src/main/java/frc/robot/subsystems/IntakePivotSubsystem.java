package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;
import frc.utils.SwerveUtils;

public class IntakePivotSubsystem extends SubsystemBase {

  private final CANSparkMax m_masterMotor = new CANSparkMax(IntakePivotConstants.kMotorCanID, MotorType.kBrushed);
  private final CANSparkMax m_followerMotor = new CANSparkMax(IntakePivotConstants.KOtherMotorCanID, MotorType.kBrushed);

  private final DutyCycleEncoder m_intakePivotMotorEncoder = new DutyCycleEncoder(IntakePivotConstants.kEncoderPort);

  private Rotation2d m_desiredAngle = Rotation2d.fromDegrees(IntakePivotConstants.kIntakeDeckPostion);

  private final PIDController pidController = new PIDController(IntakePivotConstants.kPivotP, 
                                                                IntakePivotConstants.kPivotI, 
                                                                IntakePivotConstants.kPivotD);


  private final ShuffleboardTab m_Tab = Shuffleboard.getTab("Intake");

  /** Creates a new IntakePivotSubsystem. */
  public IntakePivotSubsystem() {
    m_intakePivotMotorEncoder.setDistancePerRotation(IntakePivotConstants.kShooterEncoderPositionFactor);

    pidController.reset();
    pidController.setSetpoint(m_desiredAngle.getDegrees());
    pidController.setTolerance(IntakePivotConstants.kAngleTolerance);

    m_masterMotor.restoreFactoryDefaults();
    m_followerMotor.restoreFactoryDefaults();

    // Limit the current so the motor doesn't oof itself
    m_masterMotor.setSmartCurrentLimit(IntakePivotConstants.kMotorSmartCurrentLimit);
    m_followerMotor.setSmartCurrentLimit(IntakePivotConstants.kMotorSmartCurrentLimit);

    // Set to brake mode
    m_masterMotor.setIdleMode(IdleMode.kBrake);
    m_followerMotor.setIdleMode(IdleMode.kBrake);

    // Set inverted (positive should be down)
    m_masterMotor.setInverted(IntakePivotConstants.kInvertMotors);
    m_followerMotor.setInverted(!IntakePivotConstants.kInvertMotors);

    // Have motors use the same can ID because it's technically slightly more efficient
    m_followerMotor.follow(m_masterMotor, true);

    // Save the motor controller's configuration
    m_masterMotor.burnFlash();
    m_followerMotor.burnFlash();

    //Shuffleboard values
    
    m_Tab.addDouble("Pivot Angle", () -> getPosition().getDegrees());

    m_Tab.addBoolean("ShooterPivot Connected", () -> m_intakePivotMotorEncoder.isConnected());
  }

  /**
   * Returns the angle the pivot is at.
   * The angle is 0 when the pivot is parallel to the ground.
   *
   * @return The current position of the module (in radians).
   */
  public Rotation2d getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    if(IntakePivotConstants.kTurningMotorEncoderInverted){
      return Rotation2d.fromRadians(-m_intakePivotMotorEncoder.getDistance() + IntakePivotConstants.kAngleOffset);
    }
    return Rotation2d.fromDegrees(SwerveUtils.angleConstrain(m_intakePivotMotorEncoder.getAbsolutePosition()*IntakePivotConstants.kShooterEncoderPositionFactor + IntakePivotConstants.kAngleOffset));
  
  }


  /**
   * Set the shooter to a specific angle (in degrees)
   * @param angle The angle to set the shooter to (in degrees)
   */
  public void setPosition(double angle) {
    m_desiredAngle = Rotation2d.fromDegrees(angle);
  }

  /**
   * Set the shooter to a specific angle 
   * @param angle The angle to set the shooter to 
   */
  public void setPosition(Rotation2d angle) {
    m_desiredAngle = angle;
  }

  /** This function is for testing purposes */
  public void setSpeed(double speed){
    m_masterMotor.set(speed);
  }

  /**
   * Returns whether the shooter pivot is at its desired position within an amount of tolerance.
   * @return true if in its desired position.
   */
  public boolean inPosition(){
    return pidController.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //manageState();    
  }

  /** Putting this in a separate function so we can comment it out easier */
  private void manageState(){
    double currentPositionDegrees = getPosition().getDegrees();

    m_masterMotor.set(-pidController.calculate(currentPositionDegrees));
  }

}

