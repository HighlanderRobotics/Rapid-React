// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendClimber extends SequentialCommandGroup {
  final ClimberSubsystem climber;
  public double degrees;
  public double feet;

  // reset the climber angle to specified degrees, then extend specified feet
  public ExtendClimber(ClimberSubsystem climber, double degrees, double feet) {
    this.climber = climber;
    addRequirements(climber);
    addCommands(
      new ParallelCommandGroup(
        new InstantCommand(() -> climber.unlockRatchet()).withTimeout(0.5),
        new ResetClimberAngle(climber)
      ),
      new InstantCommand(() -> climber.setClimberAngle(degrees))
        .withInterrupt(() -> Math.abs(climber.getClimberAngle() - degrees) < 1.0),
      new InstantCommand(() -> climber.setDistance(feet))
        .withInterrupt(() -> Math.abs(climber.getDistance() - feet) < 0.1)
    );
  }

  // Stop the motor at the end (even though the default command should take over)
  @Override
  public void end(boolean interrupted) {
    climber.extensionMotor.set(TalonFXControlMode.PercentOutput, 0);
    // probably doesn't do anything but who knows
    super.end(interrupted);
  }
}
