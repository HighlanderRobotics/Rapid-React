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
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**A full sequence to line up with the hub, spin up the flywheel and align the hood, and shoot both balls. */
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
  LEDSubsystem ledSubsystem,
  XboxController controller) {
    addCommands(
      new AutoAim(visionSubsystem, drivetrainSubsystem, controller), // Line up with target (left to right)
      new WaitCommand(0.15) // Wait a little bit before spinning up to allow some time to align, since at the edge of the limelight lens distortion makes distance measurements unreliable
      .andThen(new RunCommand(() -> shooterSubsystem.setTargetRPM(visionSubsystem.getTargetRPM()), shooterSubsystem)), // Set the flywheel rpm based on distance
      new RunCommand(() -> hoodSubsystem.setSetpoint(visionSubsystem.getTargetHoodAngle()), hoodSubsystem), // Align the hood based on distance
      new SequentialCommandGroup(
        new InstantCommand(() -> printTime("started")), // Print time for debug purposes
        new WaitUntilCommand(visionSubsystem::pointingAtTarget) // Waits for pointing at hub (left to right)
          .andThen(new InstantCommand(() -> printTime("autoaim"))), // Print time for debug purposes
        new ParallelCommandGroup(
          new WaitUntilCommand(shooterSubsystem::isRPMInRange) // Then wait for the shooter to be at the right RPM
            .andThen(new InstantCommand(() -> printTime("RPM"))), // Print time for debug purposes
          new WaitUntilCommand(hoodSubsystem::atTargetAngle) // Then wait for the hood to be at the right angle
            .andThen(new InstantCommand(() -> printTime("hood"))) // Print time for debug purposes
        )
        .deadlineWith(new RunCommand(() -> routingSubsystem.setInnerFeederRPM(-1000), routingSubsystem).withTimeout(0.15) // Run the routing backwards to prevent balls from touching the flywheel during spin-up
          .andThen(new RunCommand(() -> routingSubsystem.setInnerFeederRPM(0), routingSubsystem))) // Stop routing after pulling ball back
        // .withTimeout(2.0)
        .raceWith(new RunCommand(() -> ledSubsystem.setRainbow(3), ledSubsystem)), // Make the LEDs flashy
        //new InstantCommand(drivetrainSubsystem::lock),
        new WaitCommand(0.2), // Wait a bit just in case stuff isnt fully ready
        new ShootTwoBalls(routingSubsystem, shooterSubsystem) // Run the routing to shoot both balls
          .raceWith(new RunCommand(() -> ledSubsystem.setRainbow(6), ledSubsystem)) // Make the LEDs flashier
        //new WaitCommand(0.5)
        
      )
    );
    
  }
}

