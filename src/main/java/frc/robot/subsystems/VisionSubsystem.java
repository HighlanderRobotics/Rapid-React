// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.ShootingLookup;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** Integrates several LimeLightSubsystems and has the shooting lookup table.
 * Somewhat redundant with LimeLightSubsystem
 */
public class VisionSubsystem extends SubsystemBase implements Loggable {
  // Both limelights. Upper limelight is not used.
  final LimeLightSubsystem upperLimeLight;
  public final LimeLightSubsystem lowerLimeLight;

  // A boolean to tell which limelight we are using. Not used.
  private boolean usingLowerLimeLight = false;

  // Current vision measurements.
  private double feetToTarget = 0.0;
  private double degreesToTarget = 0;

  // An interpolating lookup table containing the flywheel velocities and hood angles needed to shoot at a given distance.
  private ShootingLookup lookup;
  
  // Current shooting characteristics from vision measurements and lookup table.
  private double targetRPM;
  private double targetHoodAngle;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(final LimeLightSubsystem upperLimeLight, final LimeLightSubsystem lowerLimeLight) {
    this.upperLimeLight = upperLimeLight;
    this.lowerLimeLight = lowerLimeLight;

    lookup = new ShootingLookup();
    
    // Inserts the shooting characteristics in (Flywheel RPM, Hood Angle Degrees) form.
    lookup.insert(0.0, new Pair<>(3000.0, 0.0));
    lookup.insert(3.0, new Pair<>(2350.0, 1.0));
    lookup.insert(4.0, new Pair<>(2350.0, 5.0));
    lookup.insert(6.0, new Pair<>(2500.0, 8.0));
    lookup.insert(8.0, new Pair<>(2650.0, 13.0));
    lookup.insert(10.0, new Pair<>(2700.0, 18.0)); // old values -> 3000/23 sometimes goes over? Better with Vaughn's hood drivetrain tape
    lookup.insert(12.0, new Pair<>(2900.0, 18.0));
    lookup.insert(14.0, new Pair<>(3000.0, 19.0));
    // lookup.insert(15.0, new Pair<>(3350.0, 27.0));//old values // 3350/27 kind of worked but not sure yet... 3300/25 was ok too?
    lookup.insert(16.0, new Pair<>(3400.0, 25.0));
    lookup.insert(18.0, new Pair<>(3900.0, 29.0));
    lookup.insert(20.0, new Pair<>(4100.0, 32.2));
  }

  /**sets the current measured distance to the target.  */ 
  // Not sure why it needs to be public (Possibly held over from testing and tuning?)
  public void setFeetToTarget(double feet) {
    feetToTarget = feet;
  }

  /**Returns current measured distance */ 
  @Log
  public double getDistanceToTarget(){
    return feetToTarget;
  }

  /**Returns current measured angle (left to right) from the target */ 
  public double getAngleToTarget(){
    return degreesToTarget;
  }

  /**Returns the current target RPM from the lookup table */ 
  public double getTargetRPM() {
    return targetRPM;
  }

  /**Returns the current target hood angle from the lookup table */ 
  public double getTargetHoodAngle() {
    return targetHoodAngle;
  }

  /**Sets which limelight should be used. Currently unessecary */
  private void chooseActiveLimeLight(){
    if (lowerLimeLight.isPointingAtTarget()){
      usingLowerLimeLight = true;
    }else{
      usingLowerLimeLight = false;
    }
  }

  /**Returns the current PID to snap to the target (left to right) */
  public double pidOutput(){
    return lowerLimeLight.pidOutput;
  }

  /**Returns true if the target is within 3 degrees from centered (left to right) */
  public boolean pointingAtTarget() {
    return Math.abs(lowerLimeLight.getHorizontalOffset()) < 3.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putBoolean("is pointing", pointingAtTarget());

    // Old code to choose limelight
    if (usingLowerLimeLight){
      degreesToTarget = lowerLimeLight.getHorizontalOffset();
    }else{
      degreesToTarget = upperLimeLight.getHorizontalOffset();  
    }

    // Updates shooting characteristic values and distance to target
    Pair<Double, Double> targets = lookup.get(lowerLimeLight.getDistance());
    feetToTarget = lowerLimeLight.getDistance();
    targetRPM = targets.getFirst();
    targetHoodAngle = targets.getSecond();
    chooseActiveLimeLight();
  }
}
