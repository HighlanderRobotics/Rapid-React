// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**Runs the routing as needed to store and reject balls. Part of the in season ball rejection effort, was later replaced with a Trigger in robotcontainer. */
public class DefaultRoutingCommand extends SequentialCommandGroup {
  /** Creates a new DefaultRoutingCommand. */
  public DefaultRoutingCommand(RoutingSubsystem routingSubsystem, IntakeSubsystem intakeSubsystem, HoodSubsystem hoodSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> routingSubsystem.runRouting(true), routingSubsystem).withInterrupt(() -> routingSubsystem.shouldRejectBall()),
      new PrintCommand("running ball rejection"),
      new ConditionalCommand(
        new ProxyScheduleCommand(new BallRejection(intakeSubsystem, routingSubsystem, shooterSubsystem)
        .raceWith(new SequentialCommandGroup(new WaitUntilCommand(() -> !routingSubsystem.lowerBeambreak.get()), new WaitCommand(1.0)))), 
        new ProxyScheduleCommand(new ShootOneBall(routingSubsystem))
          .raceWith(new RunCommand(() -> {hoodSubsystem.setSetpoint(0);}, hoodSubsystem)), 
        () -> routingSubsystem.upperBeambreak.get()
    ));
  }
}
