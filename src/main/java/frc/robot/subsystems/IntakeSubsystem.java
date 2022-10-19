// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Falcon;

/** Contains the intake motor and solenoid. */
public class IntakeSubsystem extends SubsystemBase {
  // The solenoid that conrols extension
  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_SOLENOID_FORWARD, Constants.INTAKE_SOLENOID_BACKWARD);
  // Falcon that controls wheels
  public final WPI_TalonFX intakeMotor; 
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // Set up falcon and solenoid
    intakeMotor = new WPI_TalonFX(Constants.INTAKE_MOTOR);
    intakeMotor.config_kP(0, 0.15, 20);
    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20.0, 40.0, 0.5));
    intakeSolenoid.set(kReverse);
  }

  /**Sets the target RPM of the motor */
  public void setIntakeRPM(double rpm){
    intakeMotor.set(TalonFXControlMode.Velocity, Falcon.rpmToTicks(rpm));
  }

  /**Toggles the intake extension: If its out it goes in, if its in it goes out */
  public void toggleIntake(){
    intakeSolenoid.toggle();
  }

  /**Sets the intake to extend */
  public void extend(){
    intakeSolenoid.set(kForward);
  }

  /**Sets the intake to retract */
  public void retract(){
    intakeSolenoid.set(kReverse);
  }
}
