// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.components.ReversibleDigitalInput;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** The new climber for Chezy Champs.
 * Contains the motor, limit switch, and solenoids.
 */
public class TelescopingClimberSubsystem extends PIDSubsystem implements Loggable {
  // The motor for the climbers extension
  WPI_TalonFX climberMotor;
  // Solenoid which holds the passive arms in
  DoubleSolenoid mantisArmSolenoid;
  // Servo which locks and unlocks the ratchet
  Servo ratchetServo;
  // Limit switch to prevent the climber from stalling while pulling down
  ReversibleDigitalInput limitSwitch;
  // Constants related to how much the climber can extend, in encoder ticks
  public static final int MAXEXTENSION = -50000;
  // Increment for adjusting the climber up and down
  public static final double INCREMENT = 10; //convertInchesToTicks(-0.001);
  /** Creates a new TelescopingClimberSubsystem. */
  public TelescopingClimberSubsystem() {
    // PID constants and other setup
    super(new PIDController(0.00003, 0.000000, 0));
    climberMotor = new WPI_TalonFX(Constants.CLIMBER_EXTENSION_MOTOR);
    mantisArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLIMBER_SOLENOID_FORWARD, Constants.CLIMBER_SOLENOID_BACKWARD);
    ratchetServo = new Servo(Constants.CLIMBER_RATCHET_SERVO);
    limitSwitch = new ReversibleDigitalInput(Constants.CLIMBER_LIMIT_SWITCH, false);
    super.enable();
  }
  
  /**Uses the PID output, preventing the climber from pulling in when the limit switch is triggered. */
  @Override
  protected void useOutput(double output, double setpoint) {
     if (output > 0 && limitSwitch.get())
     {
      climberMotor.set(ControlMode.PercentOutput, 0);
     }else{
      climberMotor.set(ControlMode.PercentOutput, output);
     }
  }

  /**Gets the current encoder position */
  @Override
  protected double getMeasurement() {
    return climberMotor.getSelectedSensorPosition();
  }

  /**Resets the climber encoder to 0 */
  public void resetClimbMotor() {
    climberMotor.setSelectedSensorPosition(0.0);
  }

  /**Sets the pistons to extend, locking the mantis arms */
  public void extendSolenoid(){
    mantisArmSolenoid.set(kForward);
  }

  /**Sets the pistons to retract, releasing the mantis arms */
  public void retractSolenoid(){
    mantisArmSolenoid.set(kReverse);
  }

  /**Returns whether the limit switch is pressed */
  @Log
  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  /**Sets the servo to unlock the ratchet, allowing the climber to extend */
  public void unlockRatchet() {
    ratchetServo.set(0.8);
  }

  /**Sets the servo to lock the ratchet, preventing the climber from extending */
  public void lockRatchet() {
    ratchetServo.set(0.4);
  }

  /**Converts inches to falcon encoder ticks */
  public static double convertInchesToTicks(double inches) {
      return inches / 0.1014 * 2048;
  }

  /**Converts falcon encoder ticks to inches */
  public static double convertTicksToInches(double ticks) {
    return ticks / 2048 * 0.1014;
  }

  /** Tells the arm to go up if down, and down if up?
  * I don't believe we use this.
  */
  public void toggleArm()
  {
    double measurement = getMeasurement();

    System.out.println("Toggling arm. measurement = " + measurement);

    if(measurement > convertInchesToTicks(-1)){
      setSetpoint(0);
      System.out.println("Going all the way down");
    }else{
      setSetpoint(MAXEXTENSION);
      System.out.println("Going all the way up");
    }
     

  }

  /**Moves  the arm down a little bit */
  public void armDown()
  {
    if (getMeasurement() - INCREMENT < 0) {
      setSetpoint(getMeasurement() - INCREMENT);
      //System.out.println("Going down.");
    }else{
      setSetpoint(0);
      //System.out.println("Going all the way down.");
    }

  }

  /**Moves the arm up a little bit */
  public void armUp()
  {
    if (getMeasurement() + INCREMENT > MAXEXTENSION){
      setSetpoint(getMeasurement() + INCREMENT);
      System.out.println("Going up.");
    }else{
      setSetpoint(MAXEXTENSION);
      System.out.println("Going all the way up.");
    }
  }
}



