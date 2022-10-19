// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
// import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SwerveController;
import io.github.oblarg.oblog.Loggable;
import static frc.robot.Constants.*;

/** Contains the SDS Mk3 Swerve drivetrain and methods for path following.
 * Based on SDS swervelib example code.
 * Worth replacing with another library since it's hard to modify the underlying sdslib code.
 */
public class DrivetrainSubsystem extends SubsystemBase implements Loggable {
  private boolean lockOut = false;

  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
          0.0955 * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = ROTATION_SPEED_MULTPILIER * MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );


  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  
 private final AHRS m_navx = new AHRS(Port.kUSB); // NavX connected over USB
 public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  // Current target speed for the drivebase
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  // Offset used to reset gyro
  private double yawOffset = 0;

  // Field widgit for dashboard
  private final Field2d m_field = new Field2d();
  

  public DrivetrainSubsystem() {
    // Makes a tab to show each swerve module
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    // Adds the heading and field widget to shuffleboard
    SmartDashboard.putNumber("Heading", 0);
    SmartDashboard.putData("Field", m_field);

    // Configure Falcon-Falcon swerve modules
    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );
  }

  /**
   * Sets the gyroscope angle to the given value. Used to reset at the start of auto.
   */
  public void resetGyroscope(double value) {
   System.out.println("reset");
   m_navx.zeroYaw();
   yawOffset = getGyroscopeRotation().getDegrees() + yawOffset - 90 + value;
  }

  /**Resets the current angle to 0 or forward. Used in teleop to reset field relative drive */
  public void zeroGyroscope(){
    resetGyroscope(0);
  }

  /**Returns the current heading. */
  public Rotation2d getGyroscopeRotation() {
   if (m_navx.isMagnetometerCalibrated()) {
     // We will only get valid fused headings if the magnetometer is calibrated
     return Rotation2d.fromDegrees(360 - m_navx.getFusedHeading() - yawOffset);
   }


   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   if(m_navx.getYaw()<0){ 
        return Rotation2d.fromDegrees(m_navx.getYaw());
   }
   else{
        return Rotation2d.fromDegrees(360 - m_navx.getYaw());
   }
}

  /**Sets the current desired speeds (x, y, theta) */
  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  /**Sets the drive base to "Lock Out", pointing the modules perpendicular to each other to make the robot hard to push */
  public void lock(){
    lockOut = true;
  }

  /**Sets the drive base to normal driving again. */
  public void unlock(){
    lockOut = false;
  }

  /**Toggles whether to "Lock Out", pointing the modules perpendicular to each other to make the robot hard to push */
  public void toggleLock(){
    lockOut = !lockOut;
  }

  /**Follows a PathPlanner path, see SwerveController command for more info */
  public Command followPathCommand(PathPlannerTrajectory path) {
    return new SequentialCommandGroup(
      new SwerveController( // Creates a Swerve controller command
        path, // The Path we want to follow
        () -> m_odometry.getPoseMeters(), // Lambda that returns the current pose as a Pose2d
        m_kinematics, // Returns the kinematics of the drive base
        // PID controllers for x, y, and theta
        new PIDController(0.5, 0.0, 0.0), // coppied from 3175 since they have a similar bot and idk where to get these values
        new PIDController(0.5, 0.0, 0.0), //was 0.0080395 
        new ProfiledPIDController(0.6, 0.0, 0.0, new Constraints(2, 2)), //was 0.003
        (SwerveModuleState[] states) -> { // Consumes the module states to set the modules moving in the directions we want
          m_chassisSpeeds = m_kinematics.toChassisSpeeds(states);
        },
        this // The drivetrain subsystem gets added as a requirement
      ),
      new InstantCommand(() -> { m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);}) // Stop at the end of the path
    );
  }

  /**Returns the state of a swerve module */
  private SwerveModuleState getModuleState(SwerveModule module){
    return new SwerveModuleState(module.getDriveVelocity(), Rotation2d.fromDegrees(Math.toDegrees(module.getSteerAngle())));
  }

  /**Runs every processor loop to set the modules to move based on the chassis speeds. */
  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);


    m_odometry.update(getGyroscopeRotation(), 
      getModuleState(m_frontLeftModule), 
      getModuleState(m_frontRightModule),
      getModuleState(m_backLeftModule),
      getModuleState(m_backRightModule));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    
    if(!lockOut){
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    } else {
        m_frontLeftModule.set(0, 45);
        m_frontRightModule.set(0, -45);
        m_backLeftModule.set(0, -45);
        m_backRightModule.set(0, 45);
    }
  
    // Update dashboard
    SmartDashboard.putNumber("heading", getGyroscopeRotation().getDegrees());

    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putData("Field", m_field);

    SmartDashboard.putNumber("X Pose", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y Pose", m_odometry.getPoseMeters().getY());
  }
}
