/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SpeedController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class SwerveModule implements Loggable{
  private static final double kWheelRadius = 0.0508;
  private static final double kCircumference = kWheelRadius * 2 * Math.PI;
  private static final double kDriveRatio = 8.16;
  private static final double kTurningRatio = 12.8;
  private static final int kEncoderResolution = 2048;
  double targetVelocity = 1 * 2048 / 600; // X RPM 

  private static final double kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration
      = 2 * Math.PI; // radians per second squared
  
  @Log
  private WPI_TalonFX m_driveMotor;
  @Log
  private WPI_TalonFX m_turningMotor;
  
  private CANCoder m_cancoder;

  private final int driveMotorChannel;
  private final int turningMotorChannel;

@Override
public String configureLogName() {
  return "SwerveModule " + driveMotorChannel + "-" + turningMotorChannel;
}


void setDrivePIDF( double p, 
 double i, 
 double d, 
 double f){
  m_driveMotor.config_kP(0, p);
  m_driveMotor.config_kI(0, i);
  m_driveMotor.config_kD(0, d);
  m_driveMotor.config_kF(0, f);
}


void setTurningPIDF( double p, 
 double i, 
 double d, 
 double f){
  m_turningMotor.config_kP(0, p);
  m_turningMotor.config_kI(0, i);
  m_turningMotor.config_kD(0, d);
  m_turningMotor.config_kF(0, f);
}


  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int cancoderChannel, double offset) {
    
    
    this.driveMotorChannel = driveMotorChannel;
    this.turningMotorChannel = turningMotorChannel;
      m_driveMotor = new WPI_TalonFX(driveMotorChannel);
      m_turningMotor = new WPI_TalonFX(turningMotorChannel);
      m_cancoder = new CANCoder(cancoderChannel);
      m_driveMotor.configFactoryDefault();
      m_turningMotor.configFactoryDefault();
      m_driveMotor.enableVoltageCompensation(true);
      m_turningMotor.enableVoltageCompensation(true);
      m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
      m_turningMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
      m_cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
      // Doesn't work, we need to use cancoders in order to get wheel position rather than motor shaft position
      // Otherwise, will rotate past one rotation, giving us a value we can't use.

      m_turningMotor.getSensorCollection().setIntegratedSensorPosition((m_cancoder.getAbsolutePosition() - offset) * kTurningRatio * (2048.0 / 360), 20);
      //m_turningMotor.getSensorCollection().setIntegratedSensorPosition(512 * kTurningRatio, 20);
      //m_turningMotor.setSensorPhase(PhaseSensor);
      //m_turningMotor.setInverted(true);
      
    
      m_turningMotor.setNeutralMode(NeutralMode.Brake);
      m_driveMotor.setNeutralMode(NeutralMode.Brake);
    setDrivePIDF(0.01, 0, 0, 0.048);
    setTurningPIDF(0.2, 0.0, 0.1, 0.048);

    // 50% power to turning - gets 10610 units/100ms
    // 50% power to driving - 10700 units/100ms



    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
   // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity()
    , getAngle());
  }

  public Rotation2d getAngle() {
    return new Rotation2d((2*Math.PI/(2048*kTurningRatio))*(m_turningMotor.getSelectedSensorPosition()%(2048*kTurningRatio)));
  }

  public double inputAngle;

  public double setpoint;

  public double inputVelocity;

    
  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getAngle());
    long nearestDegree = Math.round(state.angle.getDegrees());
    
    double setTurnValue = (2048/360.0)*nearestDegree;

    // Calculate the drive output from the drive PID controller.
    //2048 encoder ticks per rotation, input is m/s, we want ticks per 100ms
    inputVelocity = 2048/(10*kCircumference)*state.speedMetersPerSecond*kDriveRatio;
    m_driveMotor.set(TalonFXControlMode.Velocity, inputVelocity);

    // Calculate the turning motor output from the turning PID controller.
    //2048 encoder ticks per rotation, 2pi radians per rotation, so the conversion factor is 2048/2pi radians
    inputAngle = nearestDegree;
    setpoint = setTurnValue*kTurningRatio;
    m_turningMotor.set(TalonFXControlMode.Position, setpoint);
    // System.out.print(inputAngle + "\t");
  }
}







