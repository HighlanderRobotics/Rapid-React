// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Falcon;

public class RoutingSubsystem extends SubsystemBase {
  DigitalInput lowerBeambreak = new DigitalInput(Constants.LOWER_BEAMBREAK);
  DigitalInput upperBeambreak = new DigitalInput(Constants.UPPER_BEAMBREAK);
  public final TalonFX innerFeeder = new TalonFX(Constants.INNER_FEEDER_MOTOR);
  public final TalonFX outerFeeder = new TalonFX(Constants.OUTER_FEEDER_MOTOR);
  PIDController innerFeederPID = new PIDController(1, 0, 0);
  PIDController outFeederPID = new PIDController(1, 0, 0);
  /** Creates a new RoutingSubsystem. */
  public RoutingSubsystem() {}
  
  public void setInnerFeederRPM(double rpm){
    innerFeeder.set(TalonFXControlMode.Velocity, Falcon.rpmToTicks(rpm));
  }
  public void setOuterFeederRPM(double rpm){
    outerFeeder.set(TalonFXControlMode.Velocity, Falcon.rpmToTicks(rpm));
  }
  public void runRouting(boolean intakeOut){
    boolean ballInLower = !lowerBeambreak.get();
    boolean ballInUpper = !upperBeambreak.get();
    System.out.println(ballInLower);
    System.out.println(ballInUpper);
    if(!ballInUpper){
      setInnerFeederRPM(500);
      System.out.println("Running inner");
    } else {
      setInnerFeederRPM(0);
      System.out.println("Not running");
    }

    if(intakeOut && !(ballInLower && ballInUpper)) {
      setOuterFeederRPM(500);
      System.out.println("Running outer");
    } else {
      setOuterFeederRPM(0);
      System.out.println("Not running");
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
