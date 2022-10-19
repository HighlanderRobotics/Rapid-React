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

  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_SOLENOID_FORWARD, Constants.INTAKE_SOLENOID_BACKWARD);
  public final WPI_TalonFX intakeMotor; 
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new WPI_TalonFX(Constants.INTAKE_MOTOR);
    intakeMotor.config_kP(0, 0.15, 20);
    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20.0, 40.0, 0.5));
    intakeSolenoid.set(kReverse);
  }

  public void setIntakeRPM(double rpm){
    intakeMotor.set(TalonFXControlMode.Velocity, Falcon.rpmToTicks(rpm));
  }

  public void toggleIntake(){
    intakeSolenoid.toggle();
  }

  public void extend(){
    intakeSolenoid.set(kForward);
  }

  public void retract(){
    intakeSolenoid.set(kReverse);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
