// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

/** Runs the climber angle motor down until it hits the limit switch to reset angle encoder. Part of the old climber. */
public class ResetClimberAngle extends CommandBase {
  final ClimberSubsystem climber;

  /** Command to reset the climber to the back limit switch, calibrating the encoder */
  public ResetClimberAngle(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  // Move the climber down (is this a good power?)
  @Override
  public void execute() {
    climber.angleMotor.set(TalonFXControlMode.PercentOutput, -0.2);
  }

  // Stop the motor at the end (even though the default command should take over)
  // and make sure the encoder resets (even though periodic should do it)
  @Override
  public void end(boolean interrupted) {
    climber.angleMotor.set(TalonFXControlMode.PercentOutput, 0);
    climber.angleMotor.getSensorCollection().setIntegratedSensorPosition(0, 100);
  }

  // Stop when the limit switch is hit
  @Override
  public boolean isFinished() {
    return climber.getClimberLimit();
  }
}
