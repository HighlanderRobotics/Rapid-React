// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto extends ParallelCommandGroup {
  /** Creates a new TwoBallAuto. */
  public TwoBallAuto(
    DrivetrainSubsystem drivetrainSubsystem, 
    HoodSubsystem hoodSubsystem, 
    ShooterSubsystem shooterSubsystem, 
    VisionSubsystem visionSubsystem, 
    RoutingSubsystem routingSubsystem,
    IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
      new RunCommand(() -> {intakeSubsystem.extend(); intakeSubsystem.setIntakeRPM(3000);}),
      new SequentialCommandGroup(
        new ParallelRaceGroup (
          new DefaultDriveCommand(drivetrainSubsystem, () -> 0, () -> -0.1, () -> 0, false),
          new WaitCommand(3),
          new WaitUntilCommand(() -> routingSubsystem.lowerBeambreak.get())
        ),
        new ShootingSequence(hoodSubsystem, shooterSubsystem, drivetrainSubsystem, visionSubsystem, routingSubsystem)
      )
    );
  }
}
