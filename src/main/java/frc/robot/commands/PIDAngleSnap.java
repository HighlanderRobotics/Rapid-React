// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** "Snaps" the drivetrain to a gyro angle. Was going to be used for climbing but I don't think it was ever run at comp. */
public class PIDAngleSnap extends PIDCommand {
  private DrivetrainSubsystem drivetrainSubsystem;
  double angle = 0;

  /** Creates a new PIDAngleSnap. */
  public PIDAngleSnap(DrivetrainSubsystem drivetrainSubsystem, double angle) {
    super(
        // The controller that the command will use
        Constants.AUTOAIM_PID_CONTROLLER,
        // This should return the measurement
        () -> drivetrainSubsystem.getGyroscopeRotation().getDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          new ChassisSpeeds(0, 0, output);
        });
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.angle = angle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drivetrainSubsystem.getGyroscopeRotation().getDegrees() - angle) < 5;
  }
}
