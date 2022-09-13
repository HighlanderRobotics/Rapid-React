// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.Falcon;

public class TestingSubsystem extends SubsystemBase {

  public final WPI_TalonFX testingMotor;


  public double rpmTarget = 200;

  
  private ShuffleboardTab tab = Shuffleboard.getTab("Testing");
  private NetworkTableEntry rpmSetpoint =
    tab.add("Set Motor RPM", 0)
    .getEntry();
  private NetworkTableEntry currentSetpointEntry =
    tab.add("789Current RPM Setpoint", rpmTarget)
    .getEntry();
  private NetworkTableEntry currentRPM =
    tab.add("Motor Current RPM", getRPM())
    .getEntry();
  private NetworkTableEntry currentMotorControllerSetpoint =
    tab.add("Current Motor controller setpoint", 0)
    .getEntry();
  private NetworkTableEntry encoder =
    tab.add("encoder", 0)
    .getEntry();

  /** Creates a new TestingSubsystem. */
  public TestingSubsystem() {
    testingMotor = new WPI_TalonFX(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rpmTarget = rpmSetpoint.getDouble(0);
    currentSetpointEntry.setDouble(rpmTarget);
    currentRPM.setDouble(getRPM());
    encoder.setDouble(testingMotor.getSelectedSensorVelocity() * (1.0/2048.0) * 600.0);
  }

  public void setTestingRPM(double rpm){
    testingMotor.set(TalonFXControlMode.Velocity, Falcon.rpmToTicks(rpm));
  }

  public double getRPM(){
    if (testingMotor != null){
       return testingMotor.getSelectedSensorVelocity()* (1.0/2048.0) * 600.0;
    } else {
      return 0;
    }
  }
}
