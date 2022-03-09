// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RouteOneBall extends CommandBase {

  RoutingSubsystem routingSubsystem;

  /** Creates a new ShootWithPause. */
  public RouteOneBall(RoutingSubsystem routingSubsystem) {
    this.routingSubsystem = routingSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(routingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    routingSubsystem.setInnerFeederRPM(500);
    routingSubsystem.setOuterFeederRPM(500);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    routingSubsystem.setInnerFeederRPM(0);
    routingSubsystem.setOuterFeederRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !routingSubsystem.upperBeambreak.get();
  }
}
