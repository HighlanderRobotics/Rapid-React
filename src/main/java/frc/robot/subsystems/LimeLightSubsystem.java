/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;



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

  
  public LimeLightSubsystem(String tableName) {
    this.tableName = tableName;
    
    camera = new PhotonCamera(tableName);

    lightOff();
  }

 
  public void defaultReadings() {
    

    //read values periodically
    x = camera.getLatestResult().getBestTarget().getYaw();
    y = camera.getLatestResult().getBestTarget().getPitch(); 
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

  public boolean isPointingAtTarget() {
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

  public Pose2d getEstimatedPose(Rotation2d gyroAngle){
    if (isPointingAtTarget){
      camera.getLatestResult().getBestTarget().getCameraToTarget();
      return PhotonUtils.estimateFieldToRobot(
        0.5588,
        2.0, //arbitrary value for now
        Math.toRadians(52.0),
        Math.toRadians(verticalOffset),
        new Rotation2d(Math.toRadians(horizontalOffset)),
        gyroAngle,
        new Pose2d(0, 0, new Rotation2d()), //arbitrary value for now
        new Transform2d(new Translation2d(-9.75, 0), new Rotation2d()));
    }
    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (camera.getLatestResult().getBestTarget() != null) {
      horizontalOffset = camera.getLatestResult().getBestTarget().getYaw();
      verticalOffset = camera.getLatestResult().getBestTarget().getPitch(); 
      isPointingAtTarget = camera.getLatestResult().hasTargets();
      pidOutput = autoAim();
    } 
    else{
      horizontalOffset = 0;
      verticalOffset = 0;
      isPointingAtTarget = false;
      pidOutput = 0;
    }

    
    
  }

  

}