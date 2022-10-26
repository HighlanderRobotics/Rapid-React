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

/** Old tape measure based climber subsystem.
 * Contains both motors, the limit switch, and ratchet servo.
 */
public class ClimberSubsystem extends SubsystemBase implements Loggable {
  // Climber motors
  public final LazyTalonFX angleMotor;
  public final LazyTalonFX extensionMotor;
  //TODO wtf is this for
  private boolean lastLimit = false;
  // Limit switch for resetting angle
  private final ReversibleDigitalInput limitSwitch;
  // Current extension target
  public double targetDistance = 0;
  // Used to lock/unlock the ratchet
  private final Servo ratchet;
  // Records current state of the climb sequence
  public static boolean extendedAndLocked = false;
  public static boolean startedRetracting = false;
  public static boolean startedExtension = false;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    angleMotor = new LazyTalonFX(Constants.CLIMBER_ANGLE_MOTOR);
    angleMotor.setInverted(true);
    angleMotor.configMotionCruiseVelocity(Falcon.rpmToTicks(400));
    angleMotor.configMotionAcceleration(Falcon.rpmToTicks(400));
    extensionMotor = new LazyTalonFX(Constants.CLIMBER_EXTENSION_MOTOR);
    extensionMotor.setInverted(true);
    extensionMotor.setSelectedSensorPosition(0);
    limitSwitch = new ReversibleDigitalInput(Constants.CLIMBER_LIMIT_SWITCH, false);
    ratchet = new Servo(Constants.CLIMBER_RATCHET_SERVO);
  }

  /**Sets the ratchet to locked, preventing extension */
  public void lockRatchet() {
    ratchet.set(0.1);
  }

  /**Unlocks the ratchet, allowing extension */
  public void unlockRatchet() {
    ratchet.set(0.3);
    extendedAndLocked = false;
  }

  /**Retract the tape if the tapes are extended and ratchet is locked */
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

  /**Sets the climber to go to the specified angle */
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

  /**Bumps the angle up a bit */
  public void increaseAngle(double amount){
    setClimberAngle(getClimberAngle() + amount);
  }

  /**Bumps the angle down a bit */
  public void decreaseAngle(double amount){
    setClimberAngle(getClimberAngle() - amount);
  }

  /**Extends slightly further out */
  public void increaseExtension(double amount){
    setDistance(getDistance() + amount);
  }
  
  /**Returns the current angle the climber is at */
  @Log
  public double getClimberAngle() {
    double ticks = angleMotor.getSelectedSensorPosition();
    // motor is inverted so this seems necessary
    return -Falcon.ticksToDegrees(ticks/110);
  }

  /**Converts inches to falcon encoder ticks */
  public static double inchesToTicks(double inches) {
    return inches / 0.1014 * 2048;
  }

  /**Converts falcon encoder ticks to inches */
  public static double ticksToInches(double ticks) {
    return ticks / 2048 * 0.1014;
  }

  /**Sets the target extension distance */
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

  /**Gets the current extension distance */
  @Log
  public double getDistance(){
    double ticks = extensionMotor.getSelectedSensorPosition();
    return -ticksToInches(ticks);
  }
  
  /**Gets whether the limit switch is triggered */
  @Log
  public boolean getClimberLimit(){
      return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Resets the angle based on the limit switch
    if(getClimberLimit() && !lastLimit) {
      angleMotor.getSensorCollection().setIntegratedSensorPosition(0, 100);
      lastLimit = true;
    } else if (!getClimberLimit() && lastLimit) {
      lastLimit = false;
    }
  }
}
