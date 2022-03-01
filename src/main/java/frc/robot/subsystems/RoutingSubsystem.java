// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RoutingSubsystem extends SubsystemBase {
  DigitalInput lowerBeambreak = new DigitalInput(Constants.LOWER_BEAMBREAK);
  DigitalInput upperBeambreak = new DigitalInput(Constants.UPPER_BEAMBREAK);
  public final TalonFX innerFeeder = new TalonFX(Constants.INNER_FEEDER_MOTOR);
  public final TalonFX outerFeeder = new TalonFX(Constants.OUTER_FEEDER_MOTOR);
  /** Creates a new RoutingSubsystem. */
  public RoutingSubsystem() {}
  
  public void setInnerFeederRPM(double rpm){
    double targetVelocity = (rpm * 2048) / 600;
    innerFeeder.set(TalonFXControlMode.Velocity, targetVelocity);
  }
  public void setOuterFeederRPM(double rpm){
    double targetVelocity = (rpm * 2048) / 600;
    outerFeeder.set(TalonFXControlMode.Velocity, targetVelocity);
  }
  public void runRouting(boolean intakeOut){
    boolean ballInLower = lowerBeambreak.get();
    boolean ballInUpper = upperBeambreak.get();
    if(!ballInUpper){
      setInnerFeederRPM(500);
    } else {
      setInnerFeederRPM(0);
    }

    if(intakeOut && !(ballInLower && ballInUpper)) {
      setOuterFeederRPM(500);
    } else {
      setOuterFeederRPM(0);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
