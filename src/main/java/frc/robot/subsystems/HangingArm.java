package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.HangingConstants;

public class HangingArm {
    private final CANSparkMax m_motor;
    private final DigitalInput m_upperLimit;
    private final DigitalInput m_lowerLimit;

    /**
     * Represents a hanging arm on the robot.
     * @param canID The can ID of the motor.
     * @param upperLimitPort The port number of the upper magnet sensor.
     * @param lowerLimitPort The port number of the lower magnet sensor.
     */
    public HangingArm(int canID, int upperLimitPort, int lowerLimitPort, boolean invertMotor){
        m_motor = new CANSparkMax(canID, MotorType.kBrushed);

        // Restore the motor controller to a known state (in case it's swapped out)
        m_motor.restoreFactoryDefaults();

        // Limit the current so the motor doesn't oof itself
        m_motor.setSmartCurrentLimit(HangingConstants.kHangingMotorCurrentLimit);

        // Set to brake mode
        m_motor.setIdleMode(IdleMode.kBrake);

        // Set inverted
        m_motor.setInverted(invertMotor);

        // Save the motor controller's configuration
        m_motor.burnFlash();

        m_lowerLimit = new DigitalInput(lowerLimitPort);
        m_upperLimit = new DigitalInput(upperLimitPort);
    }

    /**
     * Set the arm motor to a certain speed. 
     * This won't allow you to raise the motor if it's fully raised & vice versa.
     * @param speed A specified speed from -1 to 1.
     */
    public void setSpeed(double speed){
        // Safety: Don't allow the motors to run beyond full extension/retraction
        if((speed > 0) && isFullyExtended()){
            m_motor.stopMotor();
        }
        else if ((speed < 0) && isFullyRetracted()){
            m_motor.stopMotor();
        }
        else {
            m_motor.set(speed);
        }
    }

    /**
     * Get the speed of the arm motor.
     * @return The motor speed from -1 to 1.
     */
    public double getSpeed(){
        return m_motor.get();
    }

    /**
     * Returns whether or not the arm has reached its max extension length.
     * @return true if the arm is fully extended. 
     */
    public boolean isFullyExtended(){
        return m_upperLimit.get();
    }

    /**
     * Returns whether or not the arm has fully retracted.
     * @return true if the arm is fully retracted. 
     */
    public boolean isFullyRetracted(){
        return m_lowerLimit.get();
    }
}
