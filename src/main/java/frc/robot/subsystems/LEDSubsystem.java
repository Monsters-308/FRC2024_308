package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LEDSubsystem extends SubsystemBase {

  public final class LEDConstants {
    public static final int ledLength = 42;
    public static final int ledPort = 9;
  }
 
  private Runnable activeFunction = this::rainbow; // Reference to the currently active LED function

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  private int m_rainbowFirstPixelHue = 0;
  private int m_policeLights = 0;
  private int m_warningLightsTimer = 0;
  private int m_greenLights = 0;

  private final DoubleSupplier controllerX;
  private final DoubleSupplier visionTargets;

  private final BooleanSupplier noteInShooter;

  public LEDSubsystem(DoubleSupplier X, BooleanSupplier noteDetect, DoubleSupplier targetDetect){
    controllerX = X;
    noteInShooter = noteDetect;
    visionTargets = targetDetect;

    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(LEDConstants.ledPort);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.ledLength);
    
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
     
  }

  @Override
  public void periodic() {    
    activeFunction.run();

    if (visionTargets.getAsDouble() == 1){
      greenTargetDetect();
    }
    else if (noteInShooter.getAsBoolean() == true){
      green();
    }

    m_rainbowFirstPixelHue = (int)Math.abs(controllerX.getAsDouble()) * 10 + m_rainbowFirstPixelHue;
  } 

  public void teal() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 0 ,180);
    }

    m_led.setData(m_ledBuffer);
  }

  public void solid(int[] RGB) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, RGB[0], RGB[1], RGB[2]);
    }

    m_led.setData(m_ledBuffer);
  }
  
  public void green() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 255, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public void orange() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 255, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public void greenAlign(int ammountGreen) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, ammountGreen, 30);
    }

    m_led.setData(m_ledBuffer);
  }

  public void red() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 0, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public void purple() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 0, 255);
    }

    m_led.setData(m_ledBuffer);
  }

  public void yellow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 255, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  // Default function
  public void rainbow() {
    if(noteInShooter.getAsBoolean()){
      green();
    }

    // else if (Math.abs(controllerX.getAsDouble()) > .8){
    //   police();    
    // }

    else{
      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setHSV(i, (m_rainbowFirstPixelHue + i) % 180, 255, 255);
      }

      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 1;
    
      m_led.setData(m_ledBuffer);
    }
  }

  public void police() {
    // Calculate the hue based on the overall policeLights value
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        if (m_policeLights > 90) {
            m_ledBuffer.setRGB(i, 0, 0, 255); // Set to blue
        } else {
            m_ledBuffer.setRGB(i, 255, 0, 0); // Set to red
        }
    }

    m_policeLights += 3;
    m_policeLights %= 180;
    m_led.setData(m_ledBuffer);
  }

  // Alternating flash between red and off
  public void warning() {
    // Calculate the color based on the overall warning lights counter.
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (m_warningLightsTimer > 90) {
        m_ledBuffer.setRGB(i, 255, 0, 0); // Set to red
      } else {
        m_ledBuffer.setRGB(i, 0, 0, 0); // turn off lights
      }
    }

    m_warningLightsTimer += 6;
    m_warningLightsTimer %= 180;

    m_led.setData(m_ledBuffer);
  }

  public void greenTargetDetect() {
    // Calculate the hue based on the overall policeLights value
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, 120, m_greenLights, 255); // Set to blue

    }

    m_greenLights += 1;
    m_greenLights %= 180;
    m_led.setData(m_ledBuffer);
  }

  public void setLEDFunction(Runnable function) {
    // Set the LED function to be called periodically
    activeFunction = function;
  }
}
