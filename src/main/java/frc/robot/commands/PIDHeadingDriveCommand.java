// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Runs the drivetrain with the heading being set by a PID. Never got it to work iirc */
public class PIDHeadingDriveCommand extends PIDCommand {
  DrivetrainSubsystem drivetrainSubsystem;
  
  // Creates a new PIDHeadingDriveCommand.
  public PIDHeadingDriveCommand(DrivetrainSubsystem drivetrainSubsystem, 
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier) {
    super(
        // The controller that the command will use
        new PIDController(0.01, 0, 0),
        // This should return the measurement
        () -> drivetrainSubsystem.getGyroscopeRotation().getDegrees(),
        // This should return the setpoint (can also be a constant)
        rotationSupplier,
        // This uses the output
        output -> new DefaultDriveCommand(drivetrainSubsystem, translationXSupplier, translationYSupplier, () -> output, true)
        );
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void execute() {
  }

  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
