// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Falcon;
import frc.robot.components.LazyTalonFX;
import frc.robot.components.PicoColorSensor;
import frc.robot.components.ReversibleDigitalInput;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** Contains the two routing motors and beambreaks. */
public class RoutingSubsystem extends SubsystemBase implements Loggable {
  // The two beambreaks used to detect when balls are stored
  public ReversibleDigitalInput lowerBeambreak = new ReversibleDigitalInput(Constants.LOWER_BEAMBREAK, true);
  public ReversibleDigitalInput upperBeambreak = new ReversibleDigitalInput(Constants.UPPER_BEAMBREAK, true);
  // The two falcons used to push balls through routing
  public final TalonFX innerFeeder = new LazyTalonFX(Constants.INNER_FEEDER_MOTOR);
  public final TalonFX outerFeeder = new LazyTalonFX(Constants.OUTER_FEEDER_MOTOR);
  // PIDS for each falcon
  PIDController innerFeederPID = new PIDController(0.05, 0.0, 0);
  PIDController outerFeederPID = new PIDController(0.05, 0.0, 0);
  // Current color detected by the color sensor
  RawColor color = new RawColor(0, 0, 0, 0);
  // Color sensor
  public final PicoColorSensor colorSensor = new PicoColorSensor();
  // Class that lets us play music on the falcons. Very important
  public final Orchestra sickPipes = new Orchestra();
  /** Creates a new RoutingSubsystem. */
  public RoutingSubsystem() {
    // Configure motors
    innerFeeder.config_kP(0, innerFeederPID.getP());
    innerFeeder.config_kI(0, innerFeederPID.getI());

    outerFeeder.config_kP(0, outerFeederPID.getP());
    outerFeeder.config_kI(0, outerFeederPID.getI());

    innerFeeder.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 10, 0.25));
    outerFeeder.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 10, 0.25));

    innerFeeder.setNeutralMode(NeutralMode.Brake);
    outerFeeder.setNeutralMode(NeutralMode.Brake);

    sickPipes.addInstrument(innerFeeder);
    sickPipes.addInstrument(outerFeeder);
 
    // Load bagpipes song for the falcons to play
    sickPipes.loadMusic(Filesystem.getDeployDirectory() + "/sickPipes.chrp");
  }

  /**Returns the current rpm of the inner routing wheel */
  public double getInnerRPM(){
    return Falcon.ticksToRPM(innerFeeder.getSelectedSensorVelocity());
  }

  /**Returns the current rpm of the outer routing wheel */
  public double getOuterRPM(){
    return Falcon.ticksToRPM(outerFeeder.getSelectedSensorVelocity());
  }
  
  /**Sets the target rpm of the inner routing wheel */
  public void setInnerFeederRPM(double rpm){
    innerFeeder.set(TalonFXControlMode.Velocity, Falcon.rpmToTicks(rpm));
  }

  /**Sets the target rpm of the outer routing whee */
  public void setOuterFeederRPM(double rpm){
    outerFeeder.set(TalonFXControlMode.Velocity, Falcon.rpmToTicks(rpm));
  }

  /**returns whether the ball in the outer routing slot should be rejected */
  public boolean shouldRejectBall(){
    if(lowerBeambreak.get()){
      if (DriverStation.getAlliance() == DriverStation.Alliance.Red){
        return (getColor().blue > getColor().red);
      }
      if (DriverStation.getAlliance() == DriverStation.Alliance.Blue){
        return (getColor().red > getColor().blue);
      }
    }
    return false;
  }

  /**Runs the routing wheels as necessary based on which slots are full */
  public void runRouting(boolean intakeOut){
    boolean ballInLower = lowerBeambreak.get();
    boolean ballInUpper = upperBeambreak.get();
    if(!ballInUpper){
      setInnerFeederRPM(600);
    } else {
      setInnerFeederRPM(0);
    }

    if(intakeOut && !(ballInLower && ballInUpper)) {
      outerFeeder.set(TalonFXControlMode.PercentOutput, 0.6);
    } else {
      setOuterFeederRPM(0);
    }
  }

  /**Gets the current amount of red detected by the color sensor */
  @Log
  public double getRed() {
    return getColor().red;
  }

  /**Gets the current amount of blue detected by the color sensor */
  @Log
  public double getBlue() {
    return getColor().blue;
  }

  /**Gets the current color that the color sensor detects */
  public frc.robot.components.PicoColorSensor.RawColor getColor(){
    return colorSensor.getRawColor0();
  }

  /**Plays the bagpipes on the routing falcons. Very important */
  public void sickPipes(){
    sickPipes.play();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
