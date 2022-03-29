// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DefaultFlywheelCommand extends CommandBase {
  ShooterSubsystem shooterSubsystem;
  RoutingSubsystem routingSubsystem;
  VisionSubsystem visionSubsystem;
  /** Creates a new DefaultFlywheelCommand. */
  public DefaultFlywheelCommand(ShooterSubsystem shooterSubsystem, RoutingSubsystem routingSubsystem, VisionSubsystem visionSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.routingSubsystem = routingSubsystem;
    this.visionSubsystem = visionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (routingSubsystem.upperBeambreak.get() && routingSubsystem.lowerBeambreak.get()) {
      shooterSubsystem.setTargetRPM(visionSubsystem.getTargetRPM());
    } else {
      shooterSubsystem.setTargetRPM(0);
    }
  }

}
