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
  int pulsingValue = 0;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    led = new AddressableLED(Constants.LED_PORT);
    // 70 leds / 2 leds per index
    buffer = new AddressableLEDBuffer(70);
    led.setLength(buffer.getLength());
    led.start();
  }

  /*
  I'm not completely sure of the LED layout but I think that they are addressedd in groups of two like this:

  18   18
  19   17
  19   17
  20   16
  20   16
    ...
  33    1
  33    1
  34    0
  34    0

  this will set the LED on both sides given the index i (0-18) from the bottom
  */
  public void setSymmetrical(int i, int h, int s, int v) {
    if (i > 34) {i = 34;}
    if (i < 0)  {i = 0;}
    buffer.setHSV(i, h, s, v);
    // for i=18 this sets it twice since it's (probably) on both sides, which is inefficient but fine
    buffer.setHSV(69 - i, h, s, v);
  }

  // set a ratio of the lights on to indicate progress like climber extension
  public void setProgress(double ratio, boolean reverse, int h, int s, int v) {
    if (ratio < 0.0) {
      ratio = 0.0;
    }

    if (ratio > 1.0) {
      ratio = 1.0;
    }

    int numLights = (int)Math.round(19.0 * ratio);
    for (int i = 0; i < numLights; i++) {
      if (!reverse) {
        setSymmetrical(i, h, s, v);
      } else {
        setSymmetrical(18 - i, h, s, v);
      }
    }
  }

  public void setSolidColor(int h, int s, int v){
    for (int i = 0; i < buffer.getLength(); i ++){
      buffer.setHSV(i, h, s, v);
    }
  }

  public void setFrontColor(int h, int s, int v){
    for (int i = 18; i <= 34; i++) {
      setSymmetrical(i, h, s, v);
    }
  }

  public void setBackColor(int h, int s, int v){
    for (int i = 0; i <= 17; i ++){
      setSymmetrical(i, h, s, v);
    }
  }

  // gap = how far between, streak = how many in a row
  public void setAlternating(int gap, int streak, int h, int s, int v) {
    for (int i = 0; i <= 34; i++) {
      if ((i / streak) % (gap + 1) == 0) {
        setSymmetrical(i, h, s, v);
      } else {
        setSymmetrical(i, 0, 0, 0);
      }
    }
  }

  public void setBlinkingColor(int h, int s, int v, double time){
    if (DriverStation.getMatchTime() % time > time/2){
      setSolidColor(h, s, v);
    } else {
      setSolidColor(0, 0, 0);
    }
  }

  public void setRainbow(int increaseAmount) {
    // For every pixel
    for (int i = 0; i <= 34; i++) {
      // Calculate hue from 0 to 180 (it wraps around)
      // the constant 2 is how much the hue changes; increase it to "compress" the rainbow more
      final int hue = (rainbowFirstPixelHue + (i * 2)) % 180;
      // Set the value
      setSymmetrical(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += increaseAmount;
    // Check bounds
    rainbowFirstPixelHue %= 180;
  }

  public void setSinePulsing(int increaseAmount) {
    // For every pixel
    for (int i = 0; i <= 34; i++) {
      // Set the value to follow a sine wave from 0 to 180, with input in degrees
      setSymmetrical(i, 150, 255, (int)Math.floor(Math.sin(Math.toDegrees(pulsingValue)) * 90 + 90));
    }

    pulsingValue += increaseAmount;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    led.setData(buffer);
  }
}
