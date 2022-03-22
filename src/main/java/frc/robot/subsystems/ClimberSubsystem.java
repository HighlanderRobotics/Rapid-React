// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.LazyTalonFX;
import frc.robot.components.ReversibleDigitalInput;

public class ClimberSubsystem extends SubsystemBase {
  private final LazyTalonFX angleMotor;
  private final LazyTalonFX extensionMotor;
  private final ReversibleDigitalInput limitSwitch;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    angleMotor = new LazyTalonFX(Constants.CLIMBER_ANGLE_MOTOR);
    extensionMotor = new LazyTalonFX(Constants.CLIMBER_EXTENSION_MOTOR);
    limitSwitch = new ReversibleDigitalInput(Constants.CLIMBER_LIMIT_SWITCH, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
