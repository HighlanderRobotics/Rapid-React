// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Instant;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

/** Add your docs here. */
public class AutonomousChooser {
    SendableChooser<Command> chooser = new SendableChooser<Command>();

    DrivetrainSubsystem drivetrainSubsystem;
    HoodSubsystem hoodSubsystem;
    ShooterSubsystem shooterSubsystem;
    LimeLightSubsystem limeLightSubsystem;
    RoutingSubsystem routingSubsystem;
    IntakeSubsystem intakeSubsystem;
    LEDSubsystem ledSubsystem;
    double startAngle = 0;

    public AutonomousChooser(
        DrivetrainSubsystem drivetrainSubsystem, 
        HoodSubsystem hoodSubsystem, 
        ShooterSubsystem shooterSubsystem, 
        LimeLightSubsystem limeLightSubsystem, 
        RoutingSubsystem routingSubsystem,
        IntakeSubsystem intakeSubsystem,
        LEDSubsystem ledSubsystem){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.limeLightSubsystem = limeLightSubsystem;
        this.routingSubsystem = routingSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.ledSubsystem = ledSubsystem;
        
        chooser.setDefaultOption("UNIVERSAL 2 BALL", new TwoBallAuto(drivetrainSubsystem, hoodSubsystem, shooterSubsystem, limeLightSubsystem, routingSubsystem, intakeSubsystem, ledSubsystem));
        chooser.addOption("NONE", new PrintCommand("owo"));
        chooser.addOption("TERMINAL 3 BALL 0 HIDE", getTerminal3Ball());
        chooser.addOption("HANGAR 2 BALL 2 HIDE", getHangar2Ball2Hide());
        chooser.addOption("TERMINAL TARMAC EDGE 3 BALL", getTarmacEdgeTerminal3Ball());
        chooser.addOption("HANGAR 3 BALL 1 HIDE", getHangar2Ball1Hide());
        chooser.addOption("CHEZY CHAMPS 4 BALL", getChezyChamps4Ball());
        chooser.addOption("CHEZY CHAMPS UNIVERSAL 3 BALL", getChezyChampsUniversal3Ball());

        SmartDashboard.putData(chooser);
    }

    public Command getAutoCommand(){
        return chooser.getSelected();
    }

    private Command shoot(double time){
      return new ShootingSequence(
                hoodSubsystem, shooterSubsystem, drivetrainSubsystem, limeLightSubsystem, routingSubsystem, ledSubsystem, new XboxController(0))
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
          shoot(2.0),
          drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Upper Red 2 Ball", 2.0, 1.0)),
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
        shoot(2.0),
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
        shoot(2.0),
        drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Lower Red 2 Ball", 2.0, 1.0))
          .andThen(new WaitCommand(0.5))
          .raceWith(runIntakeAndRouting()),
        shoot(2.5),
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
        shoot(2.0),
        drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Upper Red Edge 2 Ball", 2.0, 1.0))
          .raceWith(runIntakeAndRouting()),
        shoot(2.0),
        drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Upper Red 3rd Ball", 2.0, 1.0))
          .raceWith(runIntakeAndRouting())
          .raceWith(new RunCommand(() -> shooterSubsystem.setTargetRPM(0), shooterSubsystem)),
        shoot()
      );
    }

    private Command getChezyChamps4Ball(){
      return new SequentialCommandGroup(
        new ResetHood(hoodSubsystem),
        resetOdo(PathPlanner.loadPath("Chezy Champs 3rd 4th Ball", 2.0, 1.0)),
        shoot(4.0),
        drivetrainSubsystem.followPathCommand(PathPlanner.loadPath("Chezy Champs 3rd 4th Ball", 2.0, 1.0))
          .raceWith(runIntakeAndRouting()),
        shoot()
      );
    }

    private Command getChezyChampsUniversal3Ball() {
      return new SequentialCommandGroup(
        new InstantCommand(() -> startAngle = drivetrainSubsystem.getGyroscopeRotation().getDegrees()),
        new ResetHood(hoodSubsystem),
        shoot(2.0),
        // new PIDAngleSnap(drivetrainSubsystem, startAngle),
        new TwoBallAuto(drivetrainSubsystem, hoodSubsystem, shooterSubsystem, limeLightSubsystem, routingSubsystem, intakeSubsystem, ledSubsystem)
      );
    }

    private Command resetOdo(PathPlannerTrajectory path){
      return new SequentialCommandGroup(
      new PrintCommand("" + path.getInitialState().holonomicRotation.getDegrees()),
      new InstantCommand(() -> drivetrainSubsystem.resetGyroscope(path.getInitialState().holonomicRotation.getDegrees())),
      new InstantCommand(() -> drivetrainSubsystem.m_poseEstimator.resetPosition(
        new Pose2d(path.getInitialState().poseMeters.getTranslation(), 
        path.getInitialState().holonomicRotation), drivetrainSubsystem.getGyroscopeRotation())));
      
    }
    
    private Command getOneBallAuto() {
      return new SequentialCommandGroup(
        new WaitCommand (1),
        new ParallelRaceGroup (
          new DefaultDriveCommand(drivetrainSubsystem, () -> -0.1 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, () -> 0, () -> 0, false),
          new WaitCommand(4),
          new RunCommand(() -> routingSubsystem.runRouting(true), routingSubsystem)
        ),
        new RunCommand(() -> routingSubsystem.runRouting(true), routingSubsystem).withTimeout(.25),
        new ShootingSequence(hoodSubsystem, shooterSubsystem, drivetrainSubsystem, limeLightSubsystem, routingSubsystem, ledSubsystem, new XboxController(0)),
        new ShootingSequence(hoodSubsystem, shooterSubsystem, drivetrainSubsystem, limeLightSubsystem, routingSubsystem, ledSubsystem, new XboxController(0))
      
      );
    }

}
