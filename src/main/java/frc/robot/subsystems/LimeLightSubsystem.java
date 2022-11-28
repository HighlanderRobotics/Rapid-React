/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.ShootingLookup;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;


public class LimeLightSubsystem extends SubsystemBase implements Loggable{
  public double horizontalOffset; 
  public double verticalOffset;
  public double areaOffset;
  NetworkTable table;
  public double x;
  public double y;
  public double pidOutput = 0;
  public boolean isPointingAtTarget;
  public PIDController limelightPID = Constants.AUTOAIM_PID_CONTROLLER;
  private ShuffleboardTab tab = Shuffleboard.getTab("Drive Readouts");
  public String tableName;
  PhotonCamera camera;
  private ShootingLookup lookup;
  PhotonPipelineResult result;


  
  public LimeLightSubsystem(String tableName) {
    this.tableName = tableName;
    
    camera = new PhotonCamera(tableName);

    lookup = new ShootingLookup();
    
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

    lightOff();
  }

  public double getRPM(){
    return lookup.get(getDistance()).getFirst();
  }
  
  public double getHoodAngle(){
    return lookup.get(getDistance()).getSecond();
  }

  /**Returns true if the target is within 3 degrees from centered (left to right) */
  public boolean pointingAtTarget() {
    return Math.abs(getHorizontalOffset()) < 3.0;
  }

  public void defaultReadings() {
    

    //read values periodically
    x = result.getBestTarget().getYaw();
    y = result.getBestTarget().getPitch(); 
  }

  public double getArea() {
    return areaOffset;
  }

  @Log
  public double getHorizontalOffset() {
    return horizontalOffset;
  }

  @Log
  public double getVerticalOffset() {
    return verticalOffset;
  }

  // distance to target in feet
  //@Log
  public double getDistance() {
    // angle is in degrees and Vaughn says it's mounted at 52 degrees
    double angle = Math.toRadians(verticalOffset + (90.0-52.0));
    // 81 inches between the limelight and tape
    double distance = 81.0 / Math.tan(angle);
    // convert to feet
    return distance / 12.0;
  }

  public void lightOn() {
    camera.setLED(VisionLEDMode.kOn);
  }

  public void lightOff() {
    camera.setLED(VisionLEDMode.kOff);
  }

  public boolean isTargetInView() {
    return isPointingAtTarget;
  }

  public double autoAim () {
    double output = limelightPID.calculate(horizontalOffset);
    return output;
  }

  public void controllerRumble(XboxController controller) {
    controller.setRumble(RumbleType.kLeftRumble, -horizontalOffset * 0.025);
    controller.setRumble(RumbleType.kRightRumble, horizontalOffset * 0.025);
  }

  /**Processes the vision result.
   * 
   * @return a pair of the list of poses from each target, and the latency of the result
   */
  public Pair<List<Pose2d>, Double> getEstimatedPose(){
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()){
      List<Pose2d> poses = new ArrayList<Pose2d>();
      for (PhotonTrackedTarget target : result.getTargets()){
        Pose3d targetPose3d = new Pose3d();
        switch (target.getFiducialId()) {
          case 0:
            {
              targetPose3d = new Pose3d(Units.feetToMeters(6), Units.feetToMeters(12), 0, new Rotation3d(0, 0, 0));
              break;
            }
          default:
            return null;
        }
        SmartDashboard.putNumber("Ambiguity", target.getPoseAmbiguity());
        if (target.getPoseAmbiguity() < 0.1) {
          
          Pose3d fieldToCamera = targetPose3d.transformBy(target.getCameraToTarget().inverse());
            
          Pose3d pose3dFlipped = fieldToCamera.transformBy(new Transform3d(new Translation3d(-0.248, 0.0, -0.5488), new Rotation3d()));//rotate PI rad around target
          Pose3d pose3d = new Pose3d(
            pose3dFlipped.getTranslation().minus(targetPose3d.getTranslation()).rotateBy(new Rotation3d(0.0, 0.0, Math.PI)).plus(targetPose3d.getTranslation()),
            pose3dFlipped.getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI)));

          Pose2d pose = new Pose2d(pose3d.getX(), pose3d.getY(), new Rotation2d(pose3d.getRotation().getAngle()));

          poses.add(pose);
        }
        return new Pair<>(poses, result.getLatencyMillis());
      }
    }
    return null;
  }

  public double getCameraResultLatency(){
    return result.getLatencyMillis();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    PhotonTrackedTarget bestLatestResult = result.getBestTarget();
    if (bestLatestResult != null) {
      horizontalOffset = bestLatestResult.getYaw();
      verticalOffset = bestLatestResult.getPitch(); 
      isPointingAtTarget = true;
      pidOutput = autoAim();
    } 
    else{
      horizontalOffset = 0;
      verticalOffset = 0;
      isPointingAtTarget = false;
      pidOutput = 0;
    }

    //log position to dash
    if (isPointingAtTarget) {
      // SmartDashboard.putNumber("Camera estimated pose X", getEstimatedPose());
      // SmartDashboard.putNumber("Camera estimated pose Y", getEstimatedPose());
    }
  }

  

}