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


public class ClimberSubsystem extends SubsystemBase implements Loggable {
  public final LazyTalonFX angleMotor;
  public final LazyTalonFX extensionMotor;
  private final LimitSwitch limitSwitch;
  public double targetDistance = 0;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    angleMotor = new LazyTalonFX(Constants.CLIMBER_ANGLE_MOTOR);
    angleMotor.setInverted(true);
    extensionMotor = new LazyTalonFX(Constants.CLIMBER_EXTENSION_MOTOR);
    extensionMotor.setInverted(true);
    limitSwitch = new LimitSwitch(Constants.CLIMBER_LIMIT_SWITCH, false);
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
    double ticks = angleMotor.getSelectedSensorPosition();
    // motor is inverted so this seems necessary
    return -Falcon.ticksToDegrees(ticks/110);
  }
  @Config
  public void setDistance(double distance) {
    // don't let it go below the previous distance
    if(distance<targetDistance) {
      distance=targetDistance;
    }
    targetDistance = distance;
    if(distance>5*12) {
      distance=5*12;
    }
    //0.1014 is the number of inches per rotation OF THE MOTOR (not of the wheel)
    double rotations = distance/0.1014;
    double ticks = rotations*2048;
    extensionMotor.set(TalonFXControlMode.Position, ticks);
  

  }
  @Log
  public double getDistance(){
    double ticks = extensionMotor.getSelectedSensorPosition();
    double rotations= ticks/2048;
    return -rotations * 0.1014;
  }
  @Log
  public boolean getClimberLimit(){
      return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(getClimberLimit()) {
      angleMotor.getSensorCollection().setIntegratedSensorPosition(0, 100); 
    }
  }
}
