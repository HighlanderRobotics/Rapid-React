// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer buffer;
  int rainbowFirstPixelHue = 0;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    led = new AddressableLED(Constants.LED_PORT);
    buffer = new AddressableLEDBuffer(70);
    led.setLength(buffer.getLength());
    led.start();
  }

  public void setSolidColor(int h, int s, int v){
    for (int i = 0; i < buffer.getLength(); i ++){
      buffer.setHSV(i, h, s, v);
    }
  }

  public void setBlinkingColor(int h, int s, int v, double time){
    if (DriverStation.getMatchTime() % time > time/2){
      setSolidColor(h, s, v);
    } else {
      setSolidColor(0, 0, 0);
    }
  }

  public void rainbow(int increaseAmount) {
    // For every pixel
    for (int i = 0; i < buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
      // Set the value
      buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += increaseAmount;
    // Check bounds
    rainbowFirstPixelHue %= 180;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    led.setData(buffer);
  }
}