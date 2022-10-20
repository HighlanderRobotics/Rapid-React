// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**Unlocks the ratcher, sets the climber to the right angle, and extends it. Part of the old climber. */
public class ExtendClimber extends SequentialCommandGroup {
  final ClimberSubsystem climber;

  // reset the climber angle to specified degrees, then extend specified feet
  public ExtendClimber(ClimberSubsystem climber, LEDSubsystem ledSubsystem, double degrees, double inches) {
    this.climber = climber;
    addRequirements(climber, ledSubsystem);
    addCommands(
      // clear leds
      new InstantCommand(() -> ledSubsystem.setSolidColor(0, 0, 0)),
      new InstantCommand(() -> climber.startedExtension = true),

      // unlock ratchet and reset angle first
      new ParallelCommandGroup(
        new RunCommand(() -> climber.unlockRatchet()).withTimeout(0.5),
        new ResetClimberAngle(climber)
      ),

      // go to the target angle
      new RunCommand(() -> climber.setClimberAngle(degrees))
        .withInterrupt(() -> Math.abs(climber.getClimberAngle() + degrees) < 1.0)
        .raceWith(new RunCommand(() -> ledSubsystem.setProgress(climber.getClimberAngle() / degrees, true, 10, 255, 255))),

      // then extend
      new RunCommand(() -> climber.setDistance(inches))
        .withInterrupt(() -> Math.abs(climber.getDistance() + inches) < 0.5)
        .raceWith(new RunCommand(() -> ledSubsystem.setProgress(climber.getDistance() / inches, false, 83, 255, 255))),

      // then lock
      new RunCommand(() -> climber.lockRatchet()).withTimeout(0.5)
    );
  }

  // Stop the motor at the end (even though the default command should take over)
  @Override
  public void end(boolean interrupted) {
    climber.extensionMotor.set(TalonFXControlMode.PercentOutput, 0);
    ClimberSubsystem.extendedAndLocked = true;
    // probably doesn't do anything but who knows
    super.end(interrupted);
  }
}
