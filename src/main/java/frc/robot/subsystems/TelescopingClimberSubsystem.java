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

public class TelescopingClimberSubsystem extends PIDSubsystem implements Loggable {
  WPI_TalonFX climberMotor;
  DoubleSolenoid mantisArmSolenoid;
  Servo ratchetServo;
  ReversibleDigitalInput limitSwitch;
  public static final int MAXEXTENSION = -50000;
  public static final double INCREMENT = 10; //convertInchesToTicks(-0.001);
  /** Creates a new TelescopingClimberSubsystem. */
  public TelescopingClimberSubsystem() {
    super(new PIDController(0.00005, 0.000000, 0));
    climberMotor = new WPI_TalonFX(Constants.CLIMBER_EXTENSION_MOTOR);
    mantisArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLIMBER_SOLENOID_FORWARD, Constants.CLIMBER_SOLENOID_BACKWARD);
    ratchetServo = new Servo(Constants.CLIMBER_RATCHET_SERVO);
    limitSwitch = new ReversibleDigitalInput(Constants.CLIMBER_LIMIT_SWITCH, false);
    super.enable();
  }
  

  @Override
  protected void useOutput(double output, double setpoint) {
     if (output > 0 && limitSwitch.get())
     {
      climberMotor.set(ControlMode.PercentOutput, 0);
     }else{
      climberMotor.set(ControlMode.PercentOutput, output);
     }
  }

  @Override
  protected double getMeasurement() {
    return climberMotor.getSelectedSensorPosition();
  }

  public void resetClimbMotor() {
    climberMotor.setSelectedSensorPosition(0.0);
  }

  public void extendSolenoid(){
    mantisArmSolenoid.set(kForward);
  }

  public void retractSolenoid(){
    mantisArmSolenoid.set(kReverse);
  }

  @Log
  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public void unlockRatchet() {
    ratchetServo.set(1);
  }

  public void lockRatchet() {
    ratchetServo.set(0);
  }

  public static double convertInchesToTicks(double inches) {
      return inches / 0.1014 * 2048;
  }

  public static double convertTicksToInches(double ticks) {
    return ticks / 2048 * 0.1014;
  }

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



