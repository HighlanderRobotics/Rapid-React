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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


/** Contains a single limelight. Could probably take over VisionSubsystem. */
public class LimeLightSubsystem extends SubsystemBase implements Loggable{
  // Current horizontal and vertical angle to target
  public double horizontalOffset; 
  public double verticalOffset;
  // Current area of target in vision
  public double areaOffset;
  // Redundant with horizontal and vertical offset
  public double x;
  public double y;
  // Output of pid to snap to target (left to right)
  public double pidOutput = 0;
  // Whether a target is in view
  public boolean isPointingAtTarget;
  // PID controller to snap to target (left to right)
  public PIDController limelightPID = Constants.AUTOAIM_PID_CONTROLLER;
  // Limelight network table
  public String tableName;

  
  public LimeLightSubsystem(String tableName) {
    this.tableName = tableName;
    lightOn();
  }

 /**Reads the network tables limelight values to x and y */
  public void defaultReadings() {
    //read values periodically
    x = NetworkTableInstance.getDefault().getTable(tableName).getEntry("tx").getDouble(0.0);
    y = NetworkTableInstance.getDefault().getTable(tableName).getEntry("ty").getDouble(0.0);
  }

  /**Returns the current area of the target */
  public double getArea() {
    return areaOffset;
  }

  /**Returns the current horizontal angle to the target */
  public double getHorizontalOffset() {
    return horizontalOffset;
  }

  /**Returns the current vertical angle to the target */
  public double getVerticalOffset() {
    return verticalOffset;
  }

  /**Calculates and returns the current distance to the target in feet */
  //@Log
  public double getDistance() {
    // angle is in degrees and Vaughn says it's mounted at 52 degrees
    double angle = Math.toRadians(verticalOffset + (90.0-52.0));
    // 81 inches between the limelight and tape
    double distance = 81.0 / Math.tan(angle);
    // convert to feet
    return distance / 12.0;
  }

  /**Turns on the limelight leds */
  public void lightOn() {
    NetworkTableInstance.getDefault().getTable(tableName).getEntry("ledMode").setNumber(0);
  }

  /**Turns off the limelight leds */
  public void lightOff() {
    NetworkTableInstance.getDefault().getTable(tableName).getEntry("ledMode").setNumber(1);
  }

  /**Returns whether a target is visible */
  public boolean isPointingAtTarget() {
    return isPointingAtTarget;
  }

  /**Calculates and returns the pid output */
  public double autoAim () {
    double output = limelightPID.calculate(horizontalOffset);
    return output;
  }

  /**Sets a controller to rumble based on how far to the left or right the target is */
  public void controllerRumble(XboxController controller) {
    controller.setRumble(RumbleType.kLeftRumble, -horizontalOffset * 0.025);
    controller.setRumble(RumbleType.kRightRumble, horizontalOffset * 0.025);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    horizontalOffset = NetworkTableInstance.getDefault().getTable(tableName).getEntry("tx").getDouble(0.0);
    verticalOffset = NetworkTableInstance.getDefault().getTable(tableName).getEntry("ty").getDouble(0.0);
    isPointingAtTarget = NetworkTableInstance.getDefault().getTable(tableName).getEntry("tv").getDouble(0.0) == 1;
    //SmartDashboard.putNumber("limelightX", horizontalOffset);
    //SmartDashboard.putNumber("limelightY", verticalOffset);
    //SmartDashboard.putNumber("limelightArea", area);
    //SmartDashboard.putBoolean("is limelight detecting target", isPointingAtTarget);
    //SmartDashboard.putNumber("PID output", pidOutput);
    pidOutput = autoAim();
    
  }

  

}