// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DefaultLedCommand extends CommandBase {
  private final LEDSubsystem ledSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final RoutingSubsystem routingSubsystem;
  /** Creates a new DefaultLedCommand. */
  public DefaultLedCommand(LEDSubsystem ledSubsystem, VisionSubsystem visionSubsystem, RoutingSubsystem routingSubsystem) {
    this.ledSubsystem = ledSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.routingSubsystem = routingSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  @Override
  public void execute() {
    // climber indicator
    if (ClimberSubsystem.extendedAndLocked) {
      if (ClimberSubsystem.startedRetracting) {
        // ready to retract; turn green
        ledSubsystem.setSolidColor(60, 255, 255);
      } else {
        // started climbing; run the RGB!
        ledSubsystem.rainbow(2);
      }
    } else {
      // otherwise show shoowing indicator with red/green for target lock
      boolean target = visionSubsystem.lowerLimeLight.isPointingAtTarget;

      // no ball
      if (!routingSubsystem.upperBeambreak.get()) {
        if (target) {
          ledSubsystem.setAlternating(85, 255, 255);
        } else {
          // crash here?
          ledSubsystem.setAlternating(0, 255, 255);
        }
      // one ball
      } else if (routingSubsystem.upperBeambreak.get() && !routingSubsystem.lowerBeambreak.get()) {
        if (target) {
          ledSubsystem.setFrontColor(85, 255, 255);
        } else {
          ledSubsystem.setFrontColor(0, 255, 255);
        }
        ledSubsystem.setBackColor(0, 0, 0);
      // both balls
      } else {
        if (target) {
          ledSubsystem.setSolidColor(85, 255, 255);
        } else {
          ledSubsystem.setSolidColor(0, 255, 255);
        }
      }
    }
  }
}
