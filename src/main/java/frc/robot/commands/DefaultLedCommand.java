// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
    if (visionSubsystem.lowerLimeLight.isPointingAtTarget){
      if (routingSubsystem.upperBeambreak.get() && routingSubsystem.lowerBeambreak.get()){
        ledSubsystem.setSolidColor(85, 255, 255);
      } else {
        ledSubsystem.setSolidColor(42, 255, 255);
      }
    } else {
      ledSubsystem.setSolidColor(0, 255, 255);
    }
  }
}
