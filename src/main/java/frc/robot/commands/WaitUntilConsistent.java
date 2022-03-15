// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterSubsystem;

public class WaitUntilConsistent extends CommandBase {

  private ShooterSubsystem shooter;
  private Timer timer = new Timer();
  private double time;

  /** Creates a WaitUntilConsistent, which waits until the shooter is consistently at RPM */
  public WaitUntilConsistent(ShooterSubsystem shooter, double time) {
    this.shooter = shooter;
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shooter.isRPMInRange()) {
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}
