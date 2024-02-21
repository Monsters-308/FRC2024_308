package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EncoderTest extends SubsystemBase {

    private final DutyCycleEncoder thing = new DutyCycleEncoder(2);
    /** Subsystem for testing rev encoder. */
    public EncoderTest() {
        Shuffleboard.getTab("Shooter").addDouble("DIO encoder", () -> thing.get());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

