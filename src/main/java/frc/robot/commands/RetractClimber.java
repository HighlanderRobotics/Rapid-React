// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class RetractClimber extends CommandBase {
  final ClimberSubsystem climber;

  /** Command to reset the hood to the back limit switch, calibrating the encoder */
  public RetractClimber(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  // Move the hood down (is this a good power?)
  @Override
  public void execute() {
    climber.extensionMotor.set(TalonFXControlMode.PercentOutput, -0.02);
  }

  // Stop the motor at the end (even though the default command should take over)
  // and make sure the encoder resets (even though periodic should do it)
  @Override
  public void end(boolean interrupted) {
    climber.extensionMotor.set(TalonFXControlMode.PercentOutput, 0);
    climber.targetDistance = 1;
    climber.setDistance(1);
  }

  // Stop when the limit switch is hit
  @Override
  public boolean isFinished() {
    return climber.getDistance() < 1.0;
  }
}
