package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;
import frc.utils.SwerveUtils;

public class IntakePivotSubsystem extends SubsystemBase {

  private final CANSparkMax m_masterMotor = new CANSparkMax(IntakePivotConstants.kMotorCanID, MotorType.kBrushed);
  private final CANSparkMax m_followerMotor = new CANSparkMax(IntakePivotConstants.KOtherMotorCanID, MotorType.kBrushed);

  private final DutyCycleEncoder m_intakePivotMotorEncoder = new DutyCycleEncoder(IntakePivotConstants.kEncoderPort);
  
  private final PIDController m_angleController = new PIDController(IntakePivotConstants.kPivotP, 
                                                                    IntakePivotConstants.kPivotI, 
                                                                    IntakePivotConstants.kPivotD);

  // Tracks what angle to keep the intake at in PID mode
  private double m_desiredAngleDegrees = IntakePivotConstants.kIntakeDownPosition;

  // Tracks whether to use PID to maintain a certain angle
  private boolean m_pidMode = true;

  private final ShuffleboardTab m_Tab = Shuffleboard.getTab("Intake");


  /** 
   * Controls the motors that rotate the intake down and up, along with the absolute
   * encoder to detect the rotation of the intake.
   */
  public IntakePivotSubsystem() {
    m_intakePivotMotorEncoder.setDistancePerRotation(IntakePivotConstants.kShooterEncoderPositionFactor);

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

    // Shuffleboard values
    m_Tab.addDouble("Pivot Angle", () -> getPosition().getDegrees());

    m_Tab.addBoolean("Encoder Connected", () -> m_intakePivotMotorEncoder.isConnected());

    // Shuffleboard widgets for manually controlling intake pivot
    m_Tab.add("IntakeDown", new InstantCommand(() -> setPosition(IntakePivotConstants.kIntakeDownPosition)));
      
    m_Tab.add("IntakeDeck", new InstantCommand(() -> setPosition(IntakePivotConstants.kIntakeDeckPosition)));
    
    m_Tab.add("IntakeUp", new InstantCommand(() -> setPosition(IntakePivotConstants.kIntakeInPosition)));

    m_Tab.add("PID Controller", m_angleController);
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
   * Sets the intake pivot to a specific speed. This will turn off PID control.
   * @param speed The speed to set the pivot to (-1 to 1).
   */
  public void setSpeed(double speed){
    m_pidMode = false;
    m_masterMotor.set(speed);
  }

  /**
   * Sets the intake pivot to go to and maintain a specific position. This enables PID control.
   * @param positionDegrees The position to set the pivot to, in degrees.
   */
  public void setPosition(double positionDegrees){
    m_pidMode = true;
    m_desiredAngleDegrees = MathUtil.clamp(positionDegrees, IntakePivotConstants.kPivotMinAngle, IntakePivotConstants.kPivotMaxAngle);
  }

  /**
   * Returns whether the shooter pivot is at its desired position within an amount of tolerance.
   * @return true if in its desired position.
   */
  public boolean inPosition(){
    double currentAngleDegrees = getPosition().getDegrees();
    
    return Math.abs(currentAngleDegrees - m_desiredAngleDegrees) < IntakePivotConstants.kAngleTolerance;
  }

  /** Stops pivot movement by setting the desired angle to the current angle. */
  public void stopMovement(){
    m_desiredAngleDegrees = getPosition().getDegrees();
  }

  @Override
  public void periodic() {
    if (m_pidMode){
      
      m_angleController.setSetpoint(m_desiredAngleDegrees);

      double currentAngleDegrees = getPosition().getDegrees();

      double currentAngleRadians = getPosition().getRadians();

      // Sinusoidal profiling to adjust for gravity
      double gravityOffset = Math.sin(currentAngleRadians) * IntakePivotConstants.kGravityOffsetMultiplier;

      // Total motor output with PID and gravity adjustment
      double totalMotorOutput = m_angleController.calculate(currentAngleDegrees) + gravityOffset;

      // DEBUG: display motor output to make sure we're not stalling it too much
      SmartDashboard.putNumber("IntakeOutput", totalMotorOutput);

      m_masterMotor.set(-totalMotorOutput);
    }

  }

}

