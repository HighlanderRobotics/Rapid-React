// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Falcon;
import frc.robot.components.LazyTalonFX;
import frc.robot.components.LimitSwitch;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;


public class ClimberSubsystem extends SubsystemBase implements Loggable {
  private final LazyTalonFX angleMotor;
  private final LazyTalonFX extensionMotor;
  private final LimitSwitch limitSwitch;
  private final Servo ratchetServo;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    angleMotor = new LazyTalonFX(Constants.CLIMBER_ANGLE_MOTOR);
    extensionMotor = new LazyTalonFX(Constants.CLIMBER_EXTENSION_MOTOR);
    limitSwitch = new LimitSwitch(Constants.CLIMBER_LIMIT_SWITCH, false);
    ratchetServo = new Servo(Constants.CLIMBER_RATCHET_SERVO);
  }

  @Config
  public void setClimberAngle(double angle){
    if(angle<0) {
      angle=0;
    }
    if(angle>62) {
      angle=62;
    }
    //110 should be the gear ratio
    double ticks = Falcon.degreesToTicks(angle) * 110;
    angleMotor.set(TalonFXControlMode.Position, ticks);
  }
  @Log
  public double getClimberAngle() {
    double ticks = angleMotor.getSensorCollection().getIntegratedSensorPosition();
    return Falcon.ticksToDegrees(ticks/110);
  }
  @Config
  public void setDistance(double distance) {
    if(distance<0) {
      distance=0;
    }
    if(distance>5*12) {
      distance=5*12;
    }
    //0.1014 is the number of inches per rotation
    //30 should be the gear ratio
    double rotations = distance/0.1014*30;
    double ticks = rotations*2048;
    extensionMotor.set(TalonFXControlMode.Position, ticks);
  

  }
  @Log
  public double getDistance(){
    double ticks = extensionMotor.getSensorCollection().getIntegratedSensorPosition();
    double rotations= ticks/2048;
    return rotations / 30 * 0.1014;
  }
  @Log
  public boolean getClimberLimit(){
      return limitSwitch.get();
  }

  public void setRatchetServo(double position){
    ratchetServo.set(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(getClimberLimit()) {
      angleMotor.getSensorCollection().setIntegratedSensorPosition(0, 100); 
    }
  }
}
