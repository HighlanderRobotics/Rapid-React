// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

/** Command to set the LEDs to a rainbow. Redundant since theres a method to do this in LEDSubsystem now. */
public class LEDRainbowDemoCommand extends CommandBase {
  LEDSubsystem ledSubsystem;
  int hue = 0;
  /** Creates a new LEDRainbowDemoCommand. */
  public LEDRainbowDemoCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledSubsystem.setSolidColor(hue, 255, 255);
    hue++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
