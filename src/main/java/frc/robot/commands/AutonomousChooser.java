// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        chooser.addOption("TERMINAL 3 BALL 0 HIDE", getTerminal3Ball());
        chooser.addOption("HANGAR 2 BALL 2 HIDE", getHangar2Ball2Hide());
        chooser.addOption("TERMINAL TARMAC EDGE 3 BALL", getTarmacEdgeTerminal3Ball());
        chooser.addOption("HANGAR 2 BALL 1 HIDE", getHangar2Ball1Hide());

        SmartDashboard.putData(chooser);
    }

    public Command getAutoCommand(){
        return chooser.getSelected();
    }

    private Command shoot(double time){
      return new ShootingSequence(
                hoodSubsystem, shooterSubsystem, drivetrainSubsystem, visionSubsystem, routingSubsystem, ledSubsystem)
                .withTimeout(time);
    }

    private Command shoot(){
      return shoot(3.0);
    }

    private Command runIntakeAndRouting() {
      return new ParallelCommandGroup(
        new WaitCommand(1.0).andThen(new RunCommand(() -> 
        {
          intakeSubsystem.extend(); 
          intakeSubsystem.setIntakeRPM(4000);
        }, intakeSubsystem)),
        (new RunCommand(() -> routingSubsystem.runRouting(true), routingSubsystem)));
    }

    private Command getTerminal3Ball(){
        return new SequentialCommandGroup(
          new ResetHood(hoodSubsystem),
          resetOdo(PathPlanner.loadPath("Upper Red 2 Ball", 2.0, 1.0)),
          drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Upper Red 2 Ball", 2.0, 1.0))
            .raceWith(runIntakeAndRouting()),
          shoot(2.0),
          drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Upper Red 3rd Ball", 2.0, 1.0))
            .raceWith(runIntakeAndRouting())
            .raceWith(new RunCommand(() -> shooterSubsystem.setTargetRPM(0), shooterSubsystem)),
          shoot());      
    }

    private Command getHangar2Ball2Hide(){
      return new SequentialCommandGroup(
        new ResetHood(hoodSubsystem),
        resetOdo(PathPlanner.loadPath("Lower Red 2 Ball", 8.0, 5.0)),
        drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Lower Red 2 Ball", 4.0, 5.0))
          .raceWith(runIntakeAndRouting()),
        shoot(2.0),
        drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Lower Red 2 Hide", 1.0, 2.0))
          .raceWith(runIntakeAndRouting())
          .raceWith(new RunCommand(() -> shooterSubsystem.setTargetRPM(0), shooterSubsystem))
      );
    }

    private Command getHangar2Ball1Hide(){
      return new SequentialCommandGroup(
        new ResetHood(hoodSubsystem),
        resetOdo(PathPlanner.loadPath("Lower Red 2 Ball", 2.0, 1.0)),
        drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Lower Red 2 Ball", 2.0, 1.0))
          .raceWith(runIntakeAndRouting()),
        shoot(2.0),
        drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Lower Red 1 Hide", 1.0, 2.0))
          .raceWith(runIntakeAndRouting())
          .raceWith(new RunCommand(() -> shooterSubsystem.setTargetRPM(0), shooterSubsystem)),
        new RunCommand(() -> {
          intakeSubsystem.extend();
          intakeSubsystem.setIntakeRPM(-2000);
          routingSubsystem.setInnerFeederRPM(-1500);  
          routingSubsystem.setOuterFeederRPM(-2000);
          shooterSubsystem.setTargetRPM(-1000);
        }, intakeSubsystem, routingSubsystem, shooterSubsystem)
        );
    }

    private Command getTarmacEdgeTerminal3Ball(){
      return new SequentialCommandGroup(
        new ResetHood(hoodSubsystem),
        resetOdo(PathPlanner.loadPath("Upper Red Edge 2 Ball", 2.0, 1.0)),
        drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Upper Red Edge 2 Ball", 2.0, 1.0))
          .raceWith(runIntakeAndRouting()),
        shoot(2.0),
        drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Upper Red 3rd Ball", 2.0, 1.0))
          .raceWith(runIntakeAndRouting())
          .raceWith(new RunCommand(() -> shooterSubsystem.setTargetRPM(0), shooterSubsystem)),
        shoot()
      );
    }
    private Command resetOdo(PathPlannerTrajectory path){
      return new SequentialCommandGroup(
      new InstantCommand(() -> drivetrainSubsystem.resetGyroscope(path.getInitialState().holonomicRotation.getDegrees())),
      new InstantCommand(() -> drivetrainSubsystem.m_odometry.resetPosition(
        new Pose2d(path.getInitialState().poseMeters.getTranslation(), 
        path.getInitialState().holonomicRotation), drivetrainSubsystem.getGyroscopeRotation())));
      
    }

}
