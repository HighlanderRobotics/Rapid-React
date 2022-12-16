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

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.Constants;
import frc.robot.commands.SwerveController;
import io.github.oblarg.oblog.Loggable;
import static frc.robot.Constants.*;

import java.util.List;
import java.util.TreeMap;

public class DrivetrainSubsystem extends SubsystemBase implements Loggable {
  private boolean lockOut = false;
  private boolean pathRunning = false;

  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
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
          //SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
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
  // FIXME Remove if you are using a Pigeon
//   private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  // FIXME Uncomment if you are using a NavX
 private final AHRS m_navx = new AHRS(Port.kUSB); // NavX connected over MXP
 private TreeMap<Double, Double> pastHeadings = new TreeMap<>();

 private final Matrix<N3, N1> odometryStateStdDevs;
 private final Matrix<N1, N1> odometryLocalMeasurementStdDevs;
 private final Matrix<N3, N1> odometryVisionMeasurementStdDevs;

 // Fuses vision measurements and wheel odometry
 public final SwerveDrivePoseEstimator m_poseEstimator;

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private double yawOffset = 0;

  private final Field2d m_field = new Field2d();
  

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    SmartDashboard.putNumber("Heading", 0);
    SmartDashboard.putData("Field", m_field);
    m_field.getObject("Latest Vision Pose").setPose(new Pose2d());
    m_field.getObject("Target").setPose(Constants.TARGET_POSE.toPose2d());

    // These matrices are in the form [x, y, theta], using the measurement units for these (ie meters, radians)
    odometryStateStdDevs = new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.02); // TODO: Find actual numbers for this
    odometryLocalMeasurementStdDevs = new MatBuilder<N1, N1>(Nat.N1(), Nat.N1()).fill(0.02); // TODO: Find actual numbers for this
    odometryVisionMeasurementStdDevs = new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(0.05, 0.05, 0.1); // TODO: Find actual numbers for this

    m_poseEstimator = new SwerveDrivePoseEstimator(Rotation2d.fromDegrees(m_navx.getAngle()), new Pose2d(), m_kinematics, odometryStateStdDevs, odometryLocalMeasurementStdDevs, odometryVisionMeasurementStdDevs);
    
    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // FIXME Setup motor configuration
    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
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
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void resetGyroscope(double value) {
    // FIXME Remove if you are using a Pigeon
//     m_pigeon.setFusedHeading(0.0);

    // FIXME Uncomment if you are using a NavX
   System.out.println("reset");
   m_navx.zeroYaw();
   
  }

  public void zeroGyroscope(){
    resetGyroscope(0);
  }

  public Rotation2d getGyroscopeRotation() {
    // FIXME Remove if you are using a Pigeon
//     return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   if(m_navx.getYaw() < 0){ 
        return Rotation2d.fromDegrees(m_navx.getYaw() + 180);
   }
   else{
        return Rotation2d.fromDegrees(360 - m_navx.getYaw() + 180);
   }
}

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public void lock(){
    lockOut = true;
  }

  public void unlock(){
    lockOut = false;
  }

  public void toggleLock(){
    lockOut = !lockOut;
  }

  public Command followPathCommand(PathPlannerTrajectory path) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> pathRunning = true),
      new SwerveController(
        path,
        () -> m_poseEstimator.getEstimatedPosition(),
        m_kinematics,
        new PIDController(0.5, 0.0,0.0 ), //coppied from 3175 since they have a similar bot and idk where to get these values
        new PIDController(0.5, 0.0, 0.0), //was 0.0080395 
        new ProfiledPIDController(0.6, 0.0, 0.0, new Constraints(2, 2)), //was 0.003
        (SwerveModuleState[] states) -> {
          m_chassisSpeeds = m_kinematics.toChassisSpeeds(states);
        },
        this
      ),
      new InstantCommand(() -> { pathRunning = false; m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);})
    );
  }

  private SwerveModuleState getModuleState(SwerveModule module){
    return new SwerveModuleState(module.getDriveVelocity(), Rotation2d.fromDegrees(Math.toDegrees(module.getSteerAngle())));
  }

  public void updateOdometry(Pair<List<Pose2d>, Double> data){
    System.out.println(data.getFirst());

    if (data != null) {
      m_field.getObject("Latest Vision Pose").setPoses(data.getFirst());
      SmartDashboard.putNumber("Latency", data.getSecond());
      for (Pose2d pose : data.getFirst()){
        resetToVision(pose);
        resetGyroscope(pose.getRotation().getDegrees());
      }
    }
  }

  public void resetToVision(Pose2d pose){
    m_poseEstimator.resetPosition(pose, getGyroscopeRotation());
  }

  public double getHeadingAtTime(double time){
    return pastHeadings.get(time);
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    pastHeadings.putIfAbsent(Timer.getFPGATimestamp(), getGyroscopeRotation().getDegrees());

    // Updates pose estimator
    m_poseEstimator.update(m_navx.getRotation2d().minus(new Rotation2d(Math.PI)), 
      getModuleState(m_frontLeftModule), 
      getModuleState(m_frontRightModule),
      getModuleState(m_backLeftModule),
      getModuleState(m_backRightModule));

    // Prevents modules from going faster than max speed
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
  

    SmartDashboard.putNumber("heading", getGyroscopeRotation().getDegrees());

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    // SmartDashboard.putData("Field", m_field);

    SmartDashboard.putNumber("X Pose", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Y Pose", m_poseEstimator.getEstimatedPosition().getY());
  }
}
