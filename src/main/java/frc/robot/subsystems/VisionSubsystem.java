// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.ShootingLookup;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class VisionSubsystem extends SubsystemBase implements Loggable {
  final LimeLightSubsystem upperLimeLight;
  final LimeLightSubsystem lowerLimeLight;

  @Log
  private boolean usingLowerLimeLight = false;

  private double feetToTarget = 0.0;
  private double degreesToTarget = 0;

  private ShootingLookup lookup;
  @Log
  private double targetRPM;
  @Log
  private double targetHoodAngle;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(final LimeLightSubsystem upperLimeLight, final LimeLightSubsystem lowerLimeLight) {
    this.upperLimeLight = upperLimeLight;
    this.lowerLimeLight = lowerLimeLight;

    lookup = new ShootingLookup();
    
    lookup.insert(0.0, new Pair<>(3000.0, 0.0));
    lookup.insert(3.0, new Pair<>(2400.0, 1.0));
    lookup.insert(4.0, new Pair<>(2400.0, 5.0));
    lookup.insert(5.0, new Pair<>(2500.0, 8.0));
    lookup.insert(10.0, new Pair<>(3000.0, 23.0)); // 3000/23 sometimes goes over? Better with Vaughn's hood drivetrain tape
    lookup.insert(15.0, new Pair<>(3350.0, 27.0)); // 3350/27 kind of worked but not sure yet... 3300/25 was ok too?
  }

  @Config
  public void setFeetToTarget(double feet) {
    feetToTarget = feet;
  }

  public double getDistanceToTarget(){
    return feetToTarget;
  }

  public double getAngleToTarget(){
    return degreesToTarget;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public double getTargetHoodAngle() {
    return targetHoodAngle;
  }

  private void chooseActiveLimeLight(){
    if (lowerLimeLight.isPointingAtTarget()){
      usingLowerLimeLight = true;
    }else{
      usingLowerLimeLight = false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //chooseActiveLimeLight();
    if (usingLowerLimeLight){
      degreesToTarget = lowerLimeLight.getHorizontalOffset();
    }else{
      degreesToTarget = upperLimeLight.getHorizontalOffset();  
    }

    Pair<Double, Double> targets = lookup.get(feetToTarget);
    targetRPM = targets.getFirst();
    targetHoodAngle = targets.getSecond();
  }
}
