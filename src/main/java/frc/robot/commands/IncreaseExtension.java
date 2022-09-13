// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class IncreaseExtension extends SequentialCommandGroup {
  /** Creates a new IncreaseExtension. */

  
  public ClimberSubsystem climber;
  public double startingPosition;
   
  public IncreaseExtension(ClimberSubsystem climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    addRequirements(climber);
    addCommands(
    new InstantCommand(climber::unlockRatchet),
    new WaitCommand(0.5),
    new RunCommand(() -> climber.increaseExtension(.5))
    .withInterrupt(() -> Math.abs(startingPosition + 0.5 - climber.getDistance())<0.2)
    .withTimeout(2.0)
    
    
    );

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPosition = climber.getDistance();
    super.initialize(); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      climber.lockRatchet();
      super.end(interrupted);
  }


}
