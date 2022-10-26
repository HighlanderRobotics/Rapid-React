// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimberSubsystem;

/** Locks the ratchet and pulls in the climber. Part of the old climber iirc */
public class RetractClimber extends SequentialCommandGroup {
  final ClimberSubsystem climber;

  // locks ratchet and retracts the climber, lifting the robot off the ground
  public RetractClimber(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);

    addCommands(
      new InstantCommand(() -> climber.lockRatchet()),
      new WaitCommand(0.5),
      new RunCommand(() -> climber.extensionMotor.set(TalonFXControlMode.PercentOutput, -0.2))
    );
  }

  // Stop the motor at the end (ratchet should hold it up)
  @Override
  public void end(boolean interrupted) {
    climber.extensionMotor.set(TalonFXControlMode.PercentOutput, 0);
    // probably doesn't do anything but who knows
    super.end(interrupted);
  }

  // Stop if it's all the way in, but it should be cancelled first
  @Override
  public boolean isFinished() {
    return climber.getDistance() >= -0.1;
  }
}
