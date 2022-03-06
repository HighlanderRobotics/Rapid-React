// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Falcon;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterSubsystem extends SubsystemBase implements Loggable {
  
  public final TalonFX flywheel;
  @Config
  private double targetRPM;
 
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    flywheel = new TalonFX(Constants.FLYWHEEL_MOTOR);
    flywheel.setNeutralMode(NeutralMode.Coast);
    // flywheel.configClosedloopRamp(5.0);
    flywheel.selectProfileSlot(0, 0);
    flywheel.config_kP(0, 0.2);
    flywheel.config_kI(0, 0.001);
    flywheel.config_kD(0, 0.0);
    flywheel.config_kF(0, 0.058);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTargetRPM(double rpm){
    //might need to be divided by 2
    targetRPM = rpm;
    flywheel.set(TalonFXControlMode.Velocity, Falcon.rpmToTicks(targetRPM));
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public boolean isRPMInRange(){
    return (Math.abs(Falcon.ticksToRPM(flywheel.getSelectedSensorVelocity()) - targetRPM)) < 200;
  }

  @Log
  public double currentRPM() {
    return Falcon.ticksToRPM(flywheel.getSelectedSensorVelocity());
  }
}
