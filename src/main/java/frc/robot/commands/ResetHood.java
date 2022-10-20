// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

/** Runs the hood down until it hits the limit switch to reset the encoder. Mainly used at the start of auto for match setup. */
public class ResetHood extends CommandBase {
  final HoodSubsystem hoodSubsystem;

  /** Command to reset the hood to the back limit switch, calibrating the encoder */
  public ResetHood(HoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(hoodSubsystem);
  }

  // Move the hood down (is this a good power?)
  @Override
  public void execute() {
    hoodSubsystem.hood.set(0.5);
  }

  // Stop the motor at the end (even though the default command should take over)
  // and make sure the encoder resets (even though periodic should do it)
  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.hood.set(0);
    hoodSubsystem.angleEncoder.reset();
  }

  // Stop when the limit switch is hit
  @Override
  public boolean isFinished() {
    return hoodSubsystem.bottomLimitSwitch.get();
  }
}
