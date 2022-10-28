// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Compressor;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutonomousChooser;
import frc.robot.commands.BallRejection;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExtendClimber;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.commands.IncreaseExtension;
import frc.robot.commands.LEDRainbowDemoCommand;
import frc.robot.commands.ResetHood;
import frc.robot.commands.RetractClimber;
import frc.robot.commands.ShootingSequence;
import frc.robot.components.Falcon;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TelescopingClimberSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import io.github.oblarg.oblog.annotations.Config;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController controller = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private ShuffleboardTab tab = Shuffleboard.getTab("Testing");
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
  private final LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem("gloworm");
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(new LimeLightSubsystem("gloworm"),
      limeLightSubsystem);
  private final RoutingSubsystem routingSubsystem = new RoutingSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final TelescopingClimberSubsystem climberSubsystem = new TelescopingClimberSubsystem();
  // private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(3.5);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.5);
  private final SlewRateLimiter flywheelLimiter = new SlewRateLimiter(3.0);

  private final AutonomousChooser chooser = new AutonomousChooser(drivetrainSubsystem, hoodSubsystem, shooterSubsystem,
      visionSubsystem, routingSubsystem, intakeSubsystem, ledSubsystem);

  private final PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

  private boolean shouldLockRatchet = true;

  double demoRate = 1.0;

  @Config
  double hoodTarget = 20.0;
  @Config
  double targetRPM = 500.0;

  // oblog setters
  @Config
  public void setHoodTarget(double newTarget) {
    hoodTarget = newTarget;
  }

  @Config
  public void setRPM(double newRPM) {
    targetRPM = newRPM;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // By default, use the joystick input to drive the drivetrain
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> -modifyAxis(strafeLimiter.calculate(-controller.getLeftX()))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(forwardLimiter.calculate(controller.getLeftY()))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyTurnAxis(controller.getRightX())
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        true));

        // This block of SmartDashboard.putData calls puts testing buttons on the smart dashboard
    SmartDashboard.putData("Demo Drive Mode", new InstantCommand(() -> demoRate = 0.5));

    SmartDashboard.putData("Reg Drive Mode", new InstantCommand(() -> demoRate = 1.0));

    SmartDashboard.putData("Sick Pipes", new RunCommand(() -> routingSubsystem.sickPipes(), routingSubsystem));
    // SmartDashboard.putData("Check path", new InstantCommand(() -> {
    // PathPlannerTrajectory path = PathPlanner.loadPath("Hub Scale Test", 0.5,
    // 0.5);
    // drivetrainSubsystem.m_odometry.resetPosition(
    // new Pose2d(path.getInitialState().poseMeters.getTranslation(),
    // path.getInitialState().holonomicRotation), new Rotation2d());
    // System.out.println(path.sample(0.0).poseMeters.getX());
    // System.out.println(drivetrainSubsystem.m_odometry.getPoseMeters().getX());
    // }));
    SmartDashboard.putNumber("Amp Draw", pdp.getTotalCurrent());

    SmartDashboard.putData("Climber to 0", new RunCommand(() -> climberSubsystem.setSetpoint(0), climberSubsystem));
    SmartDashboard.putData("Climber to 50,000",
        new RunCommand(() -> climberSubsystem.setSetpoint(-50000), climberSubsystem));
    SmartDashboard.putData("Climber to 12 inches", new RunCommand(
        () -> climberSubsystem.setSetpoint(TelescopingClimberSubsystem.convertInchesToTicks(-22)), climberSubsystem));
    SmartDashboard.putData("LED Demo", new LEDRainbowDemoCommand(ledSubsystem));
    SmartDashboard.putData("Ratchet Unlock",
        new RunCommand(() -> climberSubsystem.unlockRatchet(), climberSubsystem));
    SmartDashboard.putData("Ratchet Lock", new RunCommand(() -> climberSubsystem.lockRatchet(), climberSubsystem));
    SmartDashboard.putData("Extend Climber Solenoid",
        new RunCommand(() -> climberSubsystem.extendSolenoid(), climberSubsystem));
    SmartDashboard.putData("Retract Climber Solenoid",
        new RunCommand(() -> climberSubsystem.retractSolenoid(), climberSubsystem));
    // Extends intake to adjust CoG and have passive arms clear
    // Then deploys mantis arms. RunCommand holds out intake afterwards, should be
    // interrupted by controller
    SmartDashboard.putData("Traverse Sequence", new SequentialCommandGroup(
        new InstantCommand(() -> {
          intakeSubsystem.extend();
          intakeSubsystem.setIntakeRPM(2000);
        }, intakeSubsystem),
        new WaitCommand(0.4),
        new InstantCommand(() -> climberSubsystem.retractSolenoid(), climberSubsystem),
        new RunCommand(() -> {
        })));
    SmartDashboard.putData("Reset Climber to 0", new InstantCommand(() -> climberSubsystem.resetClimbMotor()));
    // SmartDashboard.putData("Aim", new RunCommand(() ->
    // hoodSubsystem.setSetpoint(visionSubsystem.getTargetHoodAngle()),
    // hoodSubsystem));
    // SmartDashboard.putData("Aim", new RunCommand(() ->
    // shooterSubsystem.setTargetRPM(visionSubsystem.getTargetRPM()),
    // shooterSubsystem));
    // SmartDashboard.putData("Manual Run Flywheel", new RunCommand(() ->
    // shooterSubsystem.setTargetRPM(targetRPM), shooterSubsystem));
    // SmartDashboard.putData("Manual Run Hood", new RunCommand(() ->
    // hoodSubsystem.setSetpoint(hoodTarget), hoodSubsystem));
    // SmartDashboard.putData("Reset Hood", new ResetHood(hoodSubsystem));
    // SmartDashboard.putData("Shoot one ball", new ShootOneBall(routingSubsystem));
    // SmartDashboard.putData("Route one ball", new RouteOneBall(routingSubsystem));
    // SmartDashboard.putData("lock ratchet", new InstantCommand(() ->
    // climberSubsystem.lockRatchet()));
    // SmartDashboard.putData("unlock ratchet", new InstantCommand(() ->
    // climberSubsystem.unlockRatchet()));
    // SmartDashboard.putData("Run Routing for Shooting", new RunCommand(() ->
    // {routingSubsystem.setOuterFeederRPM(700);
    // routingSubsystem.setInnerFeederRPM(500);}, routingSubsystem));
    // SmartDashboard.putData("Shoot two balls", new ShootTwoBalls(routingSubsystem,
    // shooterSubsystem));
    // SmartDashboard.putData("Extend Intake", new RunCommand(() ->
    // intakeSubsystem.extend(), intakeSubsystem));
    // SmartDashboard.putData("Shoot",
    // new ParallelCommandGroup(new SequentialCommandGroup(
    // new WaitUntilCommand(m_shooterSubsystem::isRPMInRange),
    // new RunCommand(() -> {m_routingSubsystem.setOuterFeederRPM(500);
    // m_routingSubsystem.setInnerFeederRPM(1000);}, m_routingSubsystem)),
    // new RunCommand(() -> m_shooterSubsystem.setTargetRPM(targetRPM),
    // m_shooterSubsystem)));
    // SmartDashboard.putData("Run Intake", new RunCommand(() ->
    // m_intakeSubsystem.setIntakeRPM(3000)));
    // SmartDashboard.putData("Toggle Intake", new InstantCommand(() ->
    // m_intakeSubsystem.toggleIntake(), m_intakeSubsystem));
    // SmartDashboard.putData("Auto Aim", new AutoAim(m_visionSubsystem,
    // m_drivetrainSubsystem));

    // climberSubsystem.setDefaultCommand(new RunCommand(() ->
    // climberSubsystem.retractIfLocked(controller.getRightTriggerAxis() * -0.6),
    // climberSubsystem));
    // By default, retract the intake and don't run the motor
    intakeSubsystem.setDefaultCommand(new RunCommand(() -> {
      intakeSubsystem.retract();
      intakeSubsystem.setIntakeRPM(0);
    }, intakeSubsystem));
    // By default, set the hood subsystem to go to an arbitrary position
    hoodSubsystem.setDefaultCommand(new RunCommand(() -> hoodSubsystem.setSetpoint(hoodTarget), hoodSubsystem));
    // Enable the hood PID
    hoodSubsystem.enable();
    // By default, spin the wheels as necessary to hold both balls
    routingSubsystem.setDefaultCommand(new RunCommand(() -> routingSubsystem.runRouting(true), routingSubsystem));
    // By default, don't spin the flywheel
    shooterSubsystem.setDefaultCommand(new RunCommand(() -> shooterSubsystem.setTargetRPM(0), shooterSubsystem));
    // By default, show the balls in the robot and the whether the target is visible
    ledSubsystem.setDefaultCommand(new DefaultLedCommand(ledSubsystem, visionSubsystem, routingSubsystem));
    // By default, lock the mantis arms, hold the climber at its starting position, and leave the ratchet open
    climberSubsystem.setDefaultCommand(new RunCommand(() -> {
      climberSubsystem.extendSolenoid();
      climberSubsystem.setSetpoint(0);
      climberSubsystem.lockRatchet();
    }, climberSubsystem));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(limeLightSubsystem::isPointingAtTarget)
      .whileActiveContinuous(
        new InstantCommand(() -> 
          drivetrainSubsystem.updateOdometry(
              limeLightSubsystem.getEstimatedPose(drivetrainSubsystem.getGyroscopeRotation()))));
    // Resets the field relative gyro heading on the swerve drive
    new Button(controller::getRightStickButton)
        .whenPressed(new InstantCommand(() -> drivetrainSubsystem.resetGyroscope(0)));
    // Runs the shooting sequence
    new Button(controller::getAButton)
            .whileHeld(new ShootingSequence(hoodSubsystem, shooterSubsystem, drivetrainSubsystem, visionSubsystem, routingSubsystem, ledSubsystem, controller)
            .alongWith(new InstantCommand(() -> {
              controller.setRumble(RumbleType.kRightRumble, 1.0);
              controller.setRumble(RumbleType.kLeftRumble, 1.0);
            }))
            .andThen(new InstantCommand(() -> {
              flywheelLimiter.reset(Falcon.ticksToRPM(shooterSubsystem.flywheel.getSelectedSensorVelocity()));
              controller.setRumble(RumbleType.kRightRumble, 0.0);
              controller.setRumble(RumbleType.kLeftRumble, 0.0);})));
    new Button(controller::getYButton)
        .whileHeld(
            new RunCommand(() -> {
              intakeSubsystem.extend();
              intakeSubsystem.setIntakeRPM(2000);
              routingSubsystem.setInnerFeederRPM(-1000);
              routingSubsystem.setOuterFeederRPM(-2000);
              shooterSubsystem.setTargetRPM(-1000);
            }, intakeSubsystem, routingSubsystem, shooterSubsystem));
    // Rejects balls out of the intake
    new Button(controller::getXButton)
        .whileHeld(new BallRejection(intakeSubsystem, routingSubsystem, shooterSubsystem));
    // Vomits balls out of the shooter
    new Button(controller::getRightBumper)
        .whileHeld(new RunCommand(() -> {
          shooterSubsystem.setTargetRPM(2000);
          routingSubsystem.setInnerFeederRPM(500);
        }));
    // Runs the intake
    new Button(controller::getLeftBumper)
        .whileHeld(new RunCommand(() -> {
          intakeSubsystem.extend();
          intakeSubsystem.setIntakeRPM(4000);
        }, intakeSubsystem));
    // Resets the hood to compensate for encoder drift or wrong startup position
    new Button(controller::getStartButton)
        .whenPressed(new ResetHood(hoodSubsystem));
    // Puts the climber up to the mid bar
    new Button(operator::getAButton)
        .toggleWhenPressed(new SequentialCommandGroup(
          new InstantCommand(() -> climberSubsystem.unlockRatchet(), climberSubsystem),
          new WaitCommand(.2),
          new RunCommand(() -> climberSubsystem.setSetpoint(TelescopingClimberSubsystem.convertInchesToTicks(-21)))));
    // Pulls the climber into the robot
    new Button(operator::getBButton)
        .toggleWhenPressed(new RunCommand(() -> {
          climberSubsystem.setSetpoint(TelescopingClimberSubsystem.convertInchesToTicks(1));
          // climberSubsystem.lockRatchet();
        },
            climberSubsystem));
    // Extends the mantis arms
    new Button(operator::getXButton)
        .whenPressed(new SequentialCommandGroup(
            new InstantCommand(() -> {
              intakeSubsystem.extend();
              intakeSubsystem.setIntakeRPM(2000);
            }, intakeSubsystem),
            new WaitCommand(0.5),
            new InstantCommand(() -> climberSubsystem.retractSolenoid(), climberSubsystem),
            new RunCommand(() -> {
            })));
    // Re extends the arms and unlocks the ratchet if the robot is ready to traverse
    new Button(operator::getYButton)
        .whenPressed(new InstantCommand(
            () -> {
              climberSubsystem.unlockRatchet();
              shouldLockRatchet = false;
              climberSubsystem.setSetpoint(TelescopingClimberSubsystem.convertInchesToTicks(-10));
            }));
    // new Button(operator::getAButton)
    // .toggleWhenPressed(new ExtendClimber(climberSubsystem, ledSubsystem, 38,
    // 20.0));
    // new Button(operator::getBButton)
    // .whenActive(new RetractClimber(climberSubsystem));
    // new Button(operator::getLeftBumper)
    // .whenPressed(new InstantCommand(() -> climberSubsystem.decreaseAngle(0.5)));
    // new Button(operator::getRightBumper)
    // .toggleWhenPressed(new IncreaseExtension(climberSubsystem));
    // new Button(operator::getStartButton)
    // .whenPressed(new InstantCommand(() -> climberSubsystem.extendedAndLocked =
    // false));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getAutoCommand();
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

    // slow it down if the climber is out
    if (!ClimberSubsystem.extendedAndLocked) {
      // Deadband
      value = deadband(value, 0.05);

      // Square the axis
      value = Math.copySign(value * value, value);
      return value;
    } else {
      value = deadband(value, 0.5);
      if (value != 0.0) {

        return Math.copySign(0.07, value);
      }
      return 0.0;
    }
  }

  // modify turn separately so it can be tuned easily if necessary
  private static double modifyTurnAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    if (!ClimberSubsystem.startedExtension) {
      return value;
    } else {
      return value * 0.2;
    }
  }

  void disabledLEDPeriodic() {
    ledSubsystem.setSolidColor(140, 255, 255);
  }

  // Called on teleop end to lock the ratchet if the robot is not traversed
  public void climberRachetTeleopExit() {
    if(shouldLockRatchet){
      climberSubsystem.lockRatchet();
    } else {
      climberSubsystem.unlockRatchet();
    }
  }
}

