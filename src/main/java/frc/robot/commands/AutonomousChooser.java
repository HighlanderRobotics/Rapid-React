// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** Add your docs here. */
public class AutonomousChooser {
    SendableChooser<Command> chooser = new SendableChooser<Command>();

    DrivetrainSubsystem drivetrainSubsystem;
    HoodSubsystem hoodSubsystem;
    ShooterSubsystem shooterSubsystem;
    VisionSubsystem visionSubsystem;
    RoutingSubsystem routingSubsystem;
    IntakeSubsystem intakeSubsystem;
    LEDSubsystem ledSubsystem;

    public AutonomousChooser(
        DrivetrainSubsystem drivetrainSubsystem, 
        HoodSubsystem hoodSubsystem, 
        ShooterSubsystem shooterSubsystem, 
        VisionSubsystem visionSubsystem, 
        RoutingSubsystem routingSubsystem,
        IntakeSubsystem intakeSubsystem,
        LEDSubsystem ledSubsystem){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.routingSubsystem = routingSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.ledSubsystem = ledSubsystem;
        
        chooser.setDefaultOption("UNIVERSAL 2 ball", new TwoBallAuto(drivetrainSubsystem, hoodSubsystem, shooterSubsystem, visionSubsystem, routingSubsystem, intakeSubsystem, ledSubsystem));
        chooser.addOption("NONE", new PrintCommand("owo"));
        chooser.addOption("RED TERMINAL 3 ball 0 hide", getRedTerminal3Ball());

        SmartDashboard.putData(chooser);
    }

    public Command getAutoCommand(){
        return chooser.getSelected();
    }

    public Command getRedTerminal3Ball(){
        return new SequentialCommandGroup(
            drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Upper Red 2 Ball", 0.5, 0.5))
            .alongWith(
              new WaitCommand(1.0).andThen(new RunCommand(() -> 
              {
                intakeSubsystem.extend(); 
                intakeSubsystem.setIntakeRPM(4000);
              },
              intakeSubsystem).withTimeout(3.0)))
              .raceWith(new RunCommand(() -> routingSubsystem.runRouting(true), routingSubsystem)),
              new ShootingSequence(
                hoodSubsystem, shooterSubsystem, drivetrainSubsystem, visionSubsystem, routingSubsystem, ledSubsystem)
                .withTimeout(3.0),
            drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Upper Red 3rd Ball", 0.5, 0.5)),
              new ShootingSequence(hoodSubsystem, shooterSubsystem, drivetrainSubsystem, visionSubsystem, routingSubsystem, ledSubsystem).withTimeout(3.0));      
    }
}
