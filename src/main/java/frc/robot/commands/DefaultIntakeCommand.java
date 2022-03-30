// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DefaultIntakeCommand extends SequentialCommandGroup {
  /** Creates a new DefaultIntakeCommand. */
  public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> {intakeSubsystem.retract(); intakeSubsystem.setIntakeRPM(3000);}, intakeSubsystem).withTimeout(1),
      new RunCommand(() -> intakeSubsystem.setIntakeRPM(0), intakeSubsystem)
    );
  }
}
