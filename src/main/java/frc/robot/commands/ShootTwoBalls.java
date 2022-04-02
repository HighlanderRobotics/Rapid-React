// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootTwoBalls extends SequentialCommandGroup {
  /** Creates a new ShootingSequence. */
  public ShootTwoBalls(RoutingSubsystem routingSubsystem, ShooterSubsystem shooter) {
    addRequirements(routingSubsystem);
    // Add your commands in the addCommands() call, e.g.
    addCommands(new ShootOneBall(routingSubsystem),
                new RunCommand(() -> routingSubsystem.runRouting(true), routingSubsystem).withInterrupt(shooter::isRPMInRange).withTimeout(2.0),
                new WaitCommand(0.1),
                new RunCommand(() -> {
                  routingSubsystem.setInnerFeederRPM(800);
                  routingSubsystem.setOuterFeederRPM(400);}, routingSubsystem));
  }
}