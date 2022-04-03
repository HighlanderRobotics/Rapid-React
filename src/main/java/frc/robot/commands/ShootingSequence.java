// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.rmi.dgc.Lease;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingSequence extends ParallelCommandGroup {

  private long lastTime = System.currentTimeMillis();

  private void printTime(String reason) {
    long currentTime = System.currentTimeMillis();
    System.out.println(reason + " took " + (currentTime - lastTime) + " milliseconds");
    lastTime = currentTime;
  }

  /** Creates a new ShootingSequence. */
  public ShootingSequence(HoodSubsystem hoodSubsystem,
  ShooterSubsystem shooterSubsystem,
  DrivetrainSubsystem drivetrainSubsystem,
  VisionSubsystem visionSubsystem,
  RoutingSubsystem routingSubsystem,
  LEDSubsystem ledSubsystem) {
    addCommands(
      new AutoAim(visionSubsystem, drivetrainSubsystem),
      new WaitCommand(0.2)
      .andThen(new RunCommand(() -> shooterSubsystem.setTargetRPM(visionSubsystem.getTargetRPM()), shooterSubsystem)),
      new RunCommand(() -> hoodSubsystem.setSetpoint(visionSubsystem.getTargetHoodAngle()), hoodSubsystem),
      new SequentialCommandGroup(
        new InstantCommand(() -> printTime("started")),
        new ParallelCommandGroup(
          new WaitUntilCommand(visionSubsystem::pointingAtTarget)
            .andThen(new InstantCommand(() -> printTime("autoaim"))),
          new WaitUntilCommand(shooterSubsystem::isRPMInRange)
            .andThen(new InstantCommand(() -> printTime("RPM"))),
          new WaitUntilCommand(hoodSubsystem::atTargetAngle)
            .andThen(new InstantCommand(() -> printTime("hood")))
        )
        .deadlineWith(new RunCommand(() -> routingSubsystem.setInnerFeederRPM(-1000), routingSubsystem).withTimeout(0.2)
          .andThen(new RunCommand(() -> routingSubsystem.setInnerFeederRPM(0), routingSubsystem)))
        // .withTimeout(2.0)
        .raceWith(new RunCommand(() -> ledSubsystem.rainbow(3), ledSubsystem)),
        //new InstantCommand(drivetrainSubsystem::lock),
        new ShootTwoBalls(routingSubsystem, shooterSubsystem)
          .alongWith(new RunCommand(() -> ledSubsystem.rainbow(6), ledSubsystem))
      )
    );
    
  }
}

