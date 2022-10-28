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
import frc.robot.subsystems.LimeLightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAimWhileMovingJank extends CommandBase{
  LimeLightSubsystem m_visionSubsystem;
  DrivetrainSubsystem m_drivetrainSubsystem;
  DoubleSupplier xIn;
  DoubleSupplier yIn;
  double endThreshold = 0.5;
  /** Creates a new AutoAim. */
  public AutoAimWhileMovingJank(LimeLightSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier xIn, DoubleSupplier yIn) {
        m_visionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        this.xIn = xIn;
        this.yIn = yIn;
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
    m_drivetrainSubsystem.drive(new ChassisSpeeds(xIn.getAsDouble(), yIn.getAsDouble(), m_visionSubsystem.autoAim()));
  }

  public void end(){
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
  }
}
