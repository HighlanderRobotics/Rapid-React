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

  /*
  I'm not completely sure of the LED layout but I think it goes something like this:

  69      0
  68      1
  67      2
  ...
  36      33
  35      34

  this will set the LED on both sides given the index i (0-34) from the bottom
  */
  public void setSymmetrical(int i, int h, int s, int v) {
    buffer.setHSV(35 - i, h, s, v);
    buffer.setHSV(i + 35, h, s, v);
  }

  public void setSolidColor(int h, int s, int v){
    for (int i = 0; i < buffer.getLength(); i ++){
      buffer.setHSV(i, h, s, v);
    }
  }

  public void setFrontColor(int h, int s, int v){
    for (int i = 0; i < 18; i++) {
      setSymmetrical(i, h, s, v);
    }
  }

  public void setBackColor(int h, int s, int v){
    for (int i = 17; i < 35; i ++){
      setSymmetrical(i, h, s, v);
    }
  }

  public void setAlternating(int h, int s, int v) {
    for (int i = 0; i < 35; i++) {
      if (i % 2 == 0) {
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

  public void rainbow(int increaseAmount) {
    // For every pixel
    for (int i = 0; i < 35; i++) {
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    led.setData(buffer);
  }
}
