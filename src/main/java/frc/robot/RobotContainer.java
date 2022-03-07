// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ResetHood;
import frc.robot.commands.ShootOneBall;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_controller = new XboxController(0);
  // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(new LimeLightSubsystem("limelight-top"), new LimeLightSubsystem("limelight-bottom"));
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
  
  // private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(); 
  private ShuffleboardTab tab = Shuffleboard.getTab("Testing");
  // private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final RoutingSubsystem m_routingSubsystem = new RoutingSubsystem();

  // private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
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

  @Log
  Command flywheelCommand = new RunCommand(() -> m_shooterSubsystem.setTargetRPM(targetRPM), m_shooterSubsystem);

  // setter for oblog
  @Config
  public void setRPM(double newRPM) {
    targetRPM = newRPM;
  }


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
    //         true
    // ));

  //  SmartDashboard.putData("Auto Aim", new AutoAim(m_limelightsubststem, m_drivetrainSubsystem, m_controller));
  //           () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
  //   ));

    // Configure the button bindings
    configureButtonBindings();
    // SmartDashboard.putData("Hood Up", new RunCommand(() -> m_shooterSubsystem.moveHood(1)));
    // SmartDashboard.putData("Hood Down", new RunCommand(() -> m_shooterSubsystem.moveHood(-1)));
    SmartDashboard.putData("Run Flywheel", new RunCommand(() -> m_shooterSubsystem.setTargetRPM(targetRPM), m_shooterSubsystem));
    SmartDashboard.putData("Run Hood", new RunCommand(() -> m_hoodSubsystem.setSetpoint(hoodTarget)));
    SmartDashboard.putData("Reset Hood", new ResetHood(m_hoodSubsystem));
    SmartDashboard.putData("Shoot one ball", new ShootOneBall(m_routingSubsystem));
    SmartDashboard.putData("Run Routing for Shooting", new RunCommand(() -> {m_routingSubsystem.setOuterFeederRPM(700); m_routingSubsystem.setInnerFeederRPM(500);}, m_routingSubsystem));
    SmartDashboard.putData("Shoot", 
    new ParallelCommandGroup(new SequentialCommandGroup(new WaitUntilCommand(m_shooterSubsystem::isRPMInRange), 
                                                        new RunCommand(() -> {m_routingSubsystem.setOuterFeederRPM(500); m_routingSubsystem.setInnerFeederRPM(1000);}, m_routingSubsystem)), 
                             new RunCommand(() -> m_shooterSubsystem.setTargetRPM(targetRPM), m_shooterSubsystem)));
    
    // SmartDashboard.putData("Lock Drivetrain", new InstantCommand(() -> m_drivetrainSubsystem.toggleLock()));
    m_shooterSubsystem.setDefaultCommand(new RunCommand(() -> m_shooterSubsystem.setTargetRPM(0), m_shooterSubsystem));
    m_hoodSubsystem.setDefaultCommand(new RunCommand(() -> m_hoodSubsystem.setSetpoint(hoodTarget), m_hoodSubsystem));
    m_hoodSubsystem.enable();
    
    m_routingSubsystem.setDefaultCommand(new RunCommand(() -> m_routingSubsystem.runRouting(true), m_routingSubsystem));

    // SmartDashboard.putData("Toggle Intake", new InstantCommand(() -> m_intakeSubsystem.toggleIntake(), m_intakeSubsystem));

    m_shooterSubsystem.setDefaultCommand(new RunCommand(() -> m_shooterSubsystem.setTargetRPM(0), m_shooterSubsystem));
    //m_hoodSubsystem.setDefaultCommand(new RunCommand(() -> m_hoodSubsystem.setSetpoint(20), m_hoodSubsystem));;
    //m_hoodSubsystem.enable();

    // Configure the button bindings
    configureButtonBindings();
    
    // SmartDashboard.putData("Auto Aim", new AutoAim(m_limelightsubststem, m_drivetrainSubsystem, m_controller));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //new Button(m_controller::getBButton)
            // No requirements because we don't need to interrupt anything
    //        .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    // new Button(m_controller::getBButton)
    //         // No requirements because we don't need to interrupt anything
    //         .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    // new Button(m_controller::getAButton)
    //         .whenPressed(new RunCommand(() -> m_intakeSubsystem.setIntakeRPM(1000)));
    // new Button(m_controller::getXButton)
    //         .whenPressed(new RunCommand(() -> m_intakeSubsystem.toggleIntake()));
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new RunCommand(() -> { System.out.println("Running autonomous"); });
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
