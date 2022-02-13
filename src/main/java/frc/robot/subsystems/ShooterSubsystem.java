// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterSubsystem extends SubsystemBase {
  public final TalonFX feeder;
  public final TalonFX flywheel;
 
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    feeder = new TalonFX(Constants.FEEDER_MOTOR);
    flywheel = new TalonFX(Constants.FLYWHEEL_MOTOR);
    
    flywheel.selectProfileSlot(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTargetRPM(double rpm){
    //might need to be divided by 2
    double targetVelocity = (rpm * 2048) / 600;
    flywheel.set(TalonFXControlMode.Velocity, targetVelocity);
  }

  // public void moveHood(double power){
  //   hood.set(power);
  // }

  public void setFeederRPM(double rpm){
    double targetVelocity = (rpm * 2048) / 600;
    feeder.set(TalonFXControlMode.Velocity, targetVelocity);
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
