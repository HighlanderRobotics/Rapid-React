// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BallRejectionSequence extends SequentialCommandGroup {  

  /** Creates a new BallRejectionSequence. */
  public BallRejectionSequence(IntakeSubsystem intakeSubsystem, RoutingSubsystem routingSubsystem, ShooterSubsystem shooterSubsystem) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        routingSubsystem.setInnerFeederRPM(0);
        routingSubsystem.setOuterFeederRPM(0);
      }),
      new WaitCommand(0.2),
      new ConditionalCommand(new BallRejection(intakeSubsystem, routingSubsystem, shooterSubsystem), new InstantCommand(), ()->routingSubsystem.shouldRejectBall())
    
    );
    
    
  }


 
}
