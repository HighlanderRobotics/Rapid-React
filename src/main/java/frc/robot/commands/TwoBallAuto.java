// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**A simple two ball auto which drives backwards with the intake out until it intakes a ball, then shoots. */
public class TwoBallAuto extends ParallelCommandGroup {
  /** Creates a new TwoBallAuto. */
  public TwoBallAuto(
    DrivetrainSubsystem drivetrainSubsystem, 
    HoodSubsystem hoodSubsystem, 
    ShooterSubsystem shooterSubsystem, 
    VisionSubsystem visionSubsystem, 
    RoutingSubsystem routingSubsystem,
    IntakeSubsystem intakeSubsystem,
    LEDSubsystem ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
      new RunCommand(() -> {intakeSubsystem.extend(); intakeSubsystem.setIntakeRPM(3000);}, intakeSubsystem),
      new SequentialCommandGroup(
        new WaitCommand (1),
        new ParallelRaceGroup (
          new DefaultDriveCommand(drivetrainSubsystem, () -> -0.1 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, () -> 0, () -> 0, false),
          new WaitCommand(3),
          new WaitUntilCommand(() -> routingSubsystem.lowerBeambreak.get()),
          new RunCommand(() -> routingSubsystem.runRouting(true), routingSubsystem)
        ),
        new RunCommand(() -> routingSubsystem.runRouting(true), routingSubsystem).withTimeout(.25),
        new ShootingSequence(hoodSubsystem, shooterSubsystem, drivetrainSubsystem, visionSubsystem, routingSubsystem, ledSubsystem, new XboxController(0)),
        new ShootingSequence(hoodSubsystem, shooterSubsystem, drivetrainSubsystem, visionSubsystem, routingSubsystem, ledSubsystem, new XboxController(0))
      )
    );
    addRequirements(drivetrainSubsystem, hoodSubsystem, shooterSubsystem, routingSubsystem, intakeSubsystem);
  }
}
