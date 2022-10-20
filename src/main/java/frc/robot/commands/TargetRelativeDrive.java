// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

/** Old attempt to drive "target relative" by having one axis control distance to target and the other control angle around it.
 * Never used, but the theory was that by having the limelight always point towards the target moving sideways would equal rotating around it.
 */
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

  }
}
