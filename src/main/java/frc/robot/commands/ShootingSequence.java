// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
  LimeLightSubsystem limelightSubsystem,
  RoutingSubsystem routingSubsystem,
  LEDSubsystem ledSubsystem,
  XboxController controller) {
    addCommands(
      new AutoAim(limelightSubsystem, drivetrainSubsystem, controller),
      new WaitCommand(0.15)
      .andThen(new RunCommand(() -> shooterSubsystem.setTargetRPM(limelightSubsystem.getRPM()), shooterSubsystem)),
      new RunCommand(() -> hoodSubsystem.setSetpoint(limelightSubsystem.getHoodAngle()), hoodSubsystem),
      new SequentialCommandGroup(
        new InstantCommand(() -> printTime("started")),
        new WaitUntilCommand(limelightSubsystem::pointingAtTarget)
          .andThen(new InstantCommand(() -> printTime("autoaim"))),
        new ParallelCommandGroup(
          new WaitUntilCommand(shooterSubsystem::isRPMInRange)
            .andThen(new InstantCommand(() -> printTime("RPM"))),
          new WaitUntilCommand(hoodSubsystem::atTargetAngle)
            .andThen(new InstantCommand(() -> printTime("hood")))
        )
        .deadlineWith(new RunCommand(() -> routingSubsystem.setInnerFeederRPM(-1000), routingSubsystem).withTimeout(0.15)
          .andThen(new RunCommand(() -> routingSubsystem.setInnerFeederRPM(0), routingSubsystem)))
        // .withTimeout(2.0)
        .raceWith(new RunCommand(() -> ledSubsystem.setRainbow(3), ledSubsystem)),
        //new InstantCommand(drivetrainSubsystem::lock),
        new WaitCommand(0.2),
        new ShootTwoBalls(routingSubsystem, shooterSubsystem)
          .raceWith(new RunCommand(() -> ledSubsystem.setRainbow(6), ledSubsystem))
        //new WaitCommand(0.5)
        
      )
    );
    
  }
}

