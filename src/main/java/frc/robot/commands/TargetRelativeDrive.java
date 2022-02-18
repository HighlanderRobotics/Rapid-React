// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.commands.DefaultDriveCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TargetRelativeDrive extends CommandBase {
  LimeLightSubsystem m_limelightSubsystem;
  DrivetrainSubsystem m_drivetrainSubsystem;
  XboxController m_controller;
  /** Creates a new TargetRelativeDrive. */
  public TargetRelativeDrive(
      LimeLightSubsystem limeLightSubsystem, 
      DrivetrainSubsystem drivetrainSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      boolean fieldRelativeTranslation) {

    m_limelightSubsystem = limeLightSubsystem;
    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
    new DefaultDriveCommand(drivetrainSubsystem, translationXSupplier, translationYSupplier, 
      () -> m_limelightSubsystem.autoAim(), fieldRelativeTranslation);

    // Use addRequirements() here to declare subsystem dependencies.
  }
}
