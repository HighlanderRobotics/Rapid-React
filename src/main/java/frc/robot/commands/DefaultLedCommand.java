// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** Sets the LEDs based on the routing and vision subsystems state for driver feedback. */
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
        // started climbing; run the RGB!
        ledSubsystem.setRainbow(2);
      } else {
        // ready to retract; turn green
        ledSubsystem.setSolidColor(60, 255, 255);
      }
    } else {
      // otherwise show shoowing indicator with red/green for target lock
      boolean target = visionSubsystem.lowerLimeLight.isPointingAtTarget;

      // no ball
      if (!routingSubsystem.upperBeambreak.get()) {
        if (target) {
          ledSubsystem.setAlternating(2, 1, 85, 255, 255);
        } else {
          // crash here?
          ledSubsystem.setAlternating(2, 1, 0, 255, 255);
        }
      // one ball
      } else if (routingSubsystem.upperBeambreak.get() && !routingSubsystem.lowerBeambreak.get()) {
        if (target) {
          ledSubsystem.setAlternating(1, 4, 85, 255, 255);
        } else {
          ledSubsystem.setAlternating(1, 4, 0, 255, 255);
        }
      // both balls
      } else {
        if (target) {
          ledSubsystem.setSolidColor(85, 255, 255);
        } else {
          ledSubsystem.setSolidColor(0, 255, 255);
        }
      }

      double time = DriverStation.getMatchTime();
      // flash purple every half second during the last 20 seconds
      if (time < 20.0 && time % 0.5 > 0.25) {
        ledSubsystem.setSolidColor(155, 255, 255); 
      }
    }
  }
}
