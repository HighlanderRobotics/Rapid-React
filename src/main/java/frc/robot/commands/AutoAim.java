// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAim extends CommandBase{
  VisionSubsystem m_visionSubsystem;
  DrivetrainSubsystem m_drivetrainSubsystem;
  XboxController m_controller;
  double endThreshold = 0.5;
  /** Creates a new AutoAim. */
  public AutoAim(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem, XboxController controller) {
        m_visionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_controller = controller;
        addRequirements(drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return Math.abs(m_visionSubsystem.getAngleToTarget()) < endThreshold;
    return false;
  }

  @Override
  public void execute () {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, m_visionSubsystem.pidOutput()));
    // m_controller.setRumble(RumbleType.kRightRumble, 1.0);
    // m_controller.setRumble(RumbleType.kLeftRumble, 1.0);
  }

  public void end(){
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
  }
}
