// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class TelescopingClimberSubsystem extends PIDSubsystem {
  WPI_TalonFX climberMotor;
  /** Creates a new TelescopingClimberSubsystem. */
  public TelescopingClimberSubsystem() {
    super(new PIDController(-0.09, 0, 0));
    climberMotor = new WPI_TalonFX(Constants.CLIMBER_EXTENSION_MOTOR);

  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // if (output > 0 && )

    climberMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  protected double getMeasurement() {
    return climberMotor.getSelectedSensorPosition();
  }
}
