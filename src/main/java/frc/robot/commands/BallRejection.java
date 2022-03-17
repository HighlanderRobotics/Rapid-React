// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RoutingSubsystem;

public class BallRejection extends CommandBase {
  IntakeSubsystem intakeSubsystem;
  RoutingSubsystem routingSubsystem;
  /** Creates a new BallRejection. */
  public BallRejection(IntakeSubsystem intakeSubsystem, RoutingSubsystem routingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.routingSubsystem = routingSubsystem;
    addRequirements(intakeSubsystem, routingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.extend();
    intakeSubsystem.setIntakeRPM(-2000);
    routingSubsystem.setOuterFeederRPM(-1000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
