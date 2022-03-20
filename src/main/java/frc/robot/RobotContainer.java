// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.text.DefaultCaret;

import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoAim;
import frc.robot.commands.BallRejection;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.PIDHeadingDriveCommand;
import frc.robot.commands.ResetHood;
import frc.robot.commands.RouteOneBall;
import frc.robot.commands.ShootOneBall;
import frc.robot.commands.ShootTwoBalls;
import frc.robot.commands.ShootingSequence;
import frc.robot.commands.TwoBallAuto;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController controller = new XboxController(0);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
  private final LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem("limelight-bottom");
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(new LimeLightSubsystem("limelight-top"), limeLightSubsystem);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(); 
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final RoutingSubsystem routingSubsystem = new RoutingSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  @Config
  double hoodTarget = 20.0;
  @Config
  double targetRPM = 500.0;
  @Config
  public void setHoodTarget(double newTarget) {
    hoodTarget = newTarget;
  }

  public void setTargetRPM(double newTarget) {
      targetRPM = newTarget;
  }

  // setter for oblog
  @Config
  public void setRPM(double newRPM) {
    targetRPM = newRPM;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            drivetrainSubsystem,
            () -> -modifyAxis(controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            true
    ));

    SmartDashboard.putData("Aim", new RunCommand(() -> hoodSubsystem.setSetpoint(visionSubsystem.getTargetHoodAngle()), hoodSubsystem));
    SmartDashboard.putData("Aim", new RunCommand(() -> shooterSubsystem.setTargetRPM(visionSubsystem.getTargetRPM()), shooterSubsystem));
    SmartDashboard.putData("Manual Run Flywheel", new RunCommand(() -> shooterSubsystem.setTargetRPM(targetRPM), shooterSubsystem));
    SmartDashboard.putData("Manual Run Hood", new RunCommand(() -> hoodSubsystem.setSetpoint(hoodTarget), hoodSubsystem));
    SmartDashboard.putData("Reset Hood", new ResetHood(hoodSubsystem));
    SmartDashboard.putData("Shoot one ball", new ShootOneBall(routingSubsystem));
    SmartDashboard.putData("Route one ball", new RouteOneBall(routingSubsystem));
    SmartDashboard.putData("Run Routing for Shooting", new RunCommand(() -> {routingSubsystem.setOuterFeederRPM(700); routingSubsystem.setInnerFeederRPM(500);}, routingSubsystem));
    SmartDashboard.putData("Shoot two balls", new ShootTwoBalls(routingSubsystem));
    SmartDashboard.putData("Extend Intake", new RunCommand(() -> intakeSubsystem.extend(), intakeSubsystem));
    SmartDashboard.putData("Shoot", 
      new ParallelCommandGroup(new SequentialCommandGroup(
        new WaitUntilCommand(shooterSubsystem::isRPMInRange), 
        new RunCommand(() -> {routingSubsystem.setOuterFeederRPM(500); routingSubsystem.setInnerFeederRPM(1000);}, routingSubsystem)), 
        new RunCommand(() -> shooterSubsystem.setTargetRPM(targetRPM), shooterSubsystem)));
    SmartDashboard.putData("Run Intake", new RunCommand(() -> intakeSubsystem.setIntakeRPM(3000)));
    SmartDashboard.putData("Toggle Intake", new InstantCommand(() -> intakeSubsystem.toggleIntake(), intakeSubsystem));
    SmartDashboard.putData("Auto Aim", new AutoAim(visionSubsystem, drivetrainSubsystem));
    SmartDashboard.putData("Reject Balls", new SequentialCommandGroup(new ExtendIntake(intakeSubsystem).withTimeout(0.5), new BallRejection(intakeSubsystem, routingSubsystem).withTimeout(1.5)));

    intakeSubsystem.setDefaultCommand(new RunCommand(() -> {intakeSubsystem.retract(); intakeSubsystem.setIntakeRPM(0);}, intakeSubsystem));
    shooterSubsystem.setDefaultCommand(new RunCommand(() -> shooterSubsystem.setTargetRPM(0), shooterSubsystem));
    hoodSubsystem.setDefaultCommand(new RunCommand(() -> hoodSubsystem.setSetpoint(hoodTarget), hoodSubsystem));
    hoodSubsystem.enable();
    // routingSubsystem.setDefaultCommand(
    //   new ConditionalCommand(
    //     new ConditionalCommand(
    //       new RunCommand(() -> routingSubsystem.runRouting(true), routingSubsystem),
    //       new ShootOneBall(routingSubsystem).alongWith(new RunCommand(() -> hoodSubsystem.setSetpoint(0))),
    //       () -> routingSubsystem.upperBeambreak.get()
    //     ),
    //     new BallRejection(intakeSubsystem, routingSubsystem),
    //     () -> routingSubsystem.rejectBall())
    //   );
    shooterSubsystem.setDefaultCommand(new RunCommand(() -> shooterSubsystem.setTargetRPM(0), shooterSubsystem));
    ledSubsystem.setDefaultCommand(new DefaultLedCommand(ledSubsystem, visionSubsystem, routingSubsystem));

    // Configure the button bindings
    configureButtonBindings();
    }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Button(controller::getBButton)
            .whenPressed(drivetrainSubsystem::zeroGyroscope);
    new Button(controller::getAButton)
            .whileHeld(new ShootingSequence(hoodSubsystem, shooterSubsystem, drivetrainSubsystem, visionSubsystem, routingSubsystem, ledSubsystem));
    new Button(controller::getYButton)
            .whenPressed(new RunCommand(() -> intakeSubsystem.setIntakeRPM(2000)));
    new Button(controller::getXButton)
            .whenPressed(new RunCommand(() -> intakeSubsystem.toggleIntake()));
    new Button(controller::getRightBumper)
            .whileHeld(new RunCommand(() -> {shooterSubsystem.setTargetRPM(2000); routingSubsystem.setInnerFeederRPM(500);}));
    new Button(controller::getLeftBumper)
            .whileHeld(new RunCommand(() -> {intakeSubsystem.extend(); intakeSubsystem.setIntakeRPM(3000);}, intakeSubsystem));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new PrintCommand("Running autonomous");
    return new TwoBallAuto(drivetrainSubsystem, hoodSubsystem, shooterSubsystem, visionSubsystem, routingSubsystem, intakeSubsystem, ledSubsystem);
  }
  
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
