// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class BallRejection extends CommandBase {
  IntakeSubsystem intakeSubsystem;
  RoutingSubsystem routingSubsystem;
  ShooterSubsystem shooterSubsystem;
  /** Creates a new BallRejection. */
  public BallRejection(IntakeSubsystem intakeSubsystem, RoutingSubsystem routingSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.routingSubsystem = routingSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(intakeSubsystem, routingSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      intakeSubsystem.extend();
      intakeSubsystem.setIntakeRPM(-2000);
      routingSubsystem.setInnerFeederRPM(-200);
      routingSubsystem.setOuterFeederRPM(-1500);
      shooterSubsystem.setTargetRPM(-1000);
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