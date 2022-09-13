// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Falcon;
import frc.robot.components.LazyTalonFX;
import frc.robot.components.ShootingLookup;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class ShooterSubsystem extends SubsystemBase implements Loggable {
  
  public final TalonFX flywheel;

  private double targetRPM = 0;

  @Config
  private double currentDistance;

  
  private double testAngle;

  private ShootingLookup lookup;

  public final KalmanFilter<N1, N1, N1> kalmanFilter;
  private final LinearSystem<N1, N1, N1> flywheelSystem;
 
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    flywheel = new LazyTalonFX(Constants.FLYWHEEL_MOTOR);
    flywheel.configFactoryDefault();
    flywheel.setNeutralMode(NeutralMode.Coast);
    // flywheel.configClosedloopRamp(5.0);
    flywheel.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 80, 0.5));
    // flywheel.configVoltageCompSaturation(12.5);
    // flywheel.enableVoltageCompensation(true);
    flywheel.selectProfileSlot(0, 0);
    flywheel.config_kP(0, 0.4);
    flywheel.config_kI(0, 0.0);
    flywheel.config_kD(0, 1.0);
    flywheel.config_kF(0, 0.058);
        //found with sys ID. 
    flywheelSystem = LinearSystemId.identifyVelocitySystem(0.11571, 0.011953);
    kalmanFilter = new KalmanFilter<>(Nat.N1(), Nat.N1(), flywheelSystem, VecBuilder.fill(3.0), VecBuilder.fill(0.01),.02);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Matrix<N1,N1> u = VecBuilder.fill(flywheel.getMotorOutputVoltage());
    kalmanFilter.predict(u, .01);
    kalmanFilter.correct(u, VecBuilder.fill(flywheel.getSelectedSensorVelocity()));
    // disable this to speed stuff up!!
    SmartDashboard.putNumber("filter output", kalmanFilter.getXhat(0));
    SmartDashboard.putNumber("RPM error", getRPMError());
    SmartDashboard.putNumber("Unfiltered RPM error", getUnfilteredRPMError());
    SmartDashboard.putBoolean("RPM in range", isRPMInRange());

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

  public double getFilteredRPM() {
    return Falcon.ticksToRPM(kalmanFilter.getXhat(0));
  }

  public double getRPMError() {
    return Math.abs(getFilteredRPM() - targetRPM);
  }

  public double getUnfilteredRPMError() {
    return Math.abs(Falcon.ticksToRPM(flywheel.getSelectedSensorVelocity()) - targetRPM);
  }

  public boolean isRPMInRange() {
    return getRPMError() < 10 && getUnfilteredRPMError() < 10;
  }

  
  public double currentRPM() {
    return Falcon.ticksToRPM(flywheel.getSelectedSensorVelocity());
  }

}
