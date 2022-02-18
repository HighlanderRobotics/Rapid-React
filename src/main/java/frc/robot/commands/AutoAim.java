// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAim extends PIDCommand {
  LimeLightSubsystem m_limeLightSubsystem;
  DrivetrainSubsystem m_drivetrainSubsystem;
  double endThreshold = 0.5;
  /** Creates a new AutoAim. */
  public AutoAim(LimeLightSubsystem limeLightSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
    super(
        // The controller that the command will use
        Constants.AUTOAIM_PID_CONTROLLER,
        // This should return the measurement
        () -> limeLightSubsystem.getHorizontalOffset(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, output));
        });
        m_limeLightSubsystem = limeLightSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        // addRequirements(drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_limeLightSubsystem.getHorizontalOffset()) < endThreshold;
  }

  public void end(){
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
  }
}
