// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class VisionSubsystem extends SubsystemBase implements Loggable {
  final LimeLightSubsystem upperLimeLight;
  final LimeLightSubsystem lowerLimeLight;

  @Log
  private boolean usingLowerLimeLight = false;
  private double feetToTarget = 0;
  private double degreesToTarget = 0;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(final LimeLightSubsystem upperLimeLight, final LimeLightSubsystem lowerLimeLight) {
    this.upperLimeLight = upperLimeLight;
    this.lowerLimeLight = lowerLimeLight;
  }

  public double getDistanceToTarget(){
    return feetToTarget;
  }

  public double getAngleToTarget(){
    return degreesToTarget;
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
    chooseActiveLimeLight();
    if (usingLowerLimeLight){
      degreesToTarget = lowerLimeLight.getHorizontalOffset();
    }else{
      degreesToTarget = upperLimeLight.getHorizontalOffset();  
    }
  }
}
