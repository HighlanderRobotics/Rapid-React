// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Falcon;
import frc.robot.components.LazyTalonFX;
import frc.robot.components.ReversibleDigitalInput;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.Servo;


public class ClimberSubsystem extends SubsystemBase implements Loggable {
  public final LazyTalonFX angleMotor;
  public final LazyTalonFX extensionMotor;
  private final ReversibleDigitalInput limitSwitch;
  public double targetDistance = 0;
  private final Servo ratchet;
  public static boolean extendedAndLocked = false;
  public static boolean startedRetracting = false;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    angleMotor = new LazyTalonFX(Constants.CLIMBER_ANGLE_MOTOR);
    angleMotor.setInverted(true);
    angleMotor.configMotionCruiseVelocity(Falcon.rpmToTicks(200));
    angleMotor.configMotionAcceleration(Falcon.rpmToTicks(100));
    extensionMotor = new LazyTalonFX(Constants.CLIMBER_EXTENSION_MOTOR);
    extensionMotor.setInverted(true);
    extensionMotor.setSelectedSensorPosition(0);
    limitSwitch = new ReversibleDigitalInput(Constants.CLIMBER_LIMIT_SWITCH, false);
    ratchet = new Servo(Constants.CLIMBER_RATCHET_SERVO);
  }

  public void lockRatchet() {
    ratchet.set(0.1);
  }

  public void unlockRatchet() {
    ratchet.set(0.3);
    extendedAndLocked = false;
  }

  public void retractIfLocked(double power) {
    // shouldn't be extending
    if (power > 0) {
      power = 0;
    }

    // ONLY go if everything is locked and extended, and don't go with super low power
    if (extendedAndLocked && power < -0.1) {
      extensionMotor.set(TalonFXControlMode.PercentOutput, power);
      startedRetracting = true;
    } else {
      // otherwise should stop
      extensionMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  @Config
  public void setClimberAngle(double angle){
    if(angle < 0) {
      angle = 0;
    }
    if(angle > 62) {
      angle = 62;
    }
    //110 should be the gear ratio
    double ticks = Falcon.degreesToTicks(angle) * 110;
    angleMotor.set(TalonFXControlMode.MotionMagic, ticks);
  }

  public void increaseAngle(double amount){
    setClimberAngle(getClimberAngle() + amount);
  }

  public void decreaseAngle(double amount){
    setClimberAngle(getClimberAngle() - amount);
  }

  @Log
  public double getClimberAngle() {
    double ticks = angleMotor.getSelectedSensorPosition();
    // motor is inverted so this seems necessary
    return -Falcon.ticksToDegrees(ticks/110);
  }

  public static double inchesToTicks(double inches) {
    return inches / 0.1014 * 2048;
  }

  public static double ticksToInches(double ticks) {
    return ticks / 2048 * 0.1014;
  }

  @Config
  public void setDistance(double distance) {
    unlockRatchet();
    // don't let it go below the previous distance
    if(distance<targetDistance) {
      distance=targetDistance;
    }
    targetDistance = distance;
    if(distance > 5 * 12) {
      distance = 5 * 12;
    }
    //0.1014 is the number of inches per rotation OF THE MOTOR (not of the wheel)
    double ticks = inchesToTicks(distance);
    extensionMotor.set(TalonFXControlMode.Position, ticks);
  }
  @Log
  public double getDistance(){
    double ticks = extensionMotor.getSelectedSensorPosition();
    return -ticksToInches(ticks);
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
