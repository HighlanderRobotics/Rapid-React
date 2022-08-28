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
    super(new PIDController(0.00001, 0, 0));
    climberMotor = new WPI_TalonFX(Constants.CLIMBER_EXTENSION_MOTOR);

    super.enable();
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // if (output > 0 && )

    climberMotor.set(ControlMode.PercentOutput, output);
    System.out.println("Output: " + output);
    System.out.println("Setpoint: " + setpoint);
    System.out.println("Current Position: " + getMeasurement());
    System.out.println("Ticks: " + getMeasurement());
    System.out.println("Ticks to Inches: " + convertTicksToInches(getMeasurement()));
  }

  @Override
  protected double getMeasurement() {
    return climberMotor.getSelectedSensorPosition();
  }

  protected double convertInchesToTicks(double inches) {
      return inches / 0.1014 * 2048;
  }

  protected double convertTicksToInches(double ticks) {
    return ticks / 2048 * 0.1014;
  }
}



