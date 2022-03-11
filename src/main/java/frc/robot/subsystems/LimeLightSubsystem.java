/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
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

  
  public LimeLightSubsystem(String tableName) {
    this.tableName = tableName;
    lightOn();
  }

 
  public void defaultReadings() {
    

    //read values periodically
    x = NetworkTableInstance.getDefault().getTable(tableName).getEntry("tx").getDouble(0.0);
    y = NetworkTableInstance.getDefault().getTable(tableName).getEntry("ty").getDouble(0.0);
    double area = NetworkTableInstance.getDefault().getTable(tableName).getEntry("ta").getDouble(0.0);


    
  }

  public void lightReadings() {
    
  }

  public double getArea() {
    return areaOffset;
  }

  public double getHorizontalOffset() {
    return horizontalOffset;
  }

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
    NetworkTableInstance.getDefault().getTable(tableName).getEntry("ledMode").setNumber(0);
  }

  public void lightOff() {
    NetworkTableInstance.getDefault().getTable(tableName).getEntry("ledMode").setNumber(1);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    horizontalOffset = NetworkTableInstance.getDefault().getTable(tableName).getEntry("tx").getDouble(0.0);
    verticalOffset = NetworkTableInstance.getDefault().getTable(tableName).getEntry("ty").getDouble(0.0);
    double area = NetworkTableInstance.getDefault().getTable(tableName).getEntry("ta").getDouble(0.0);
    isPointingAtTarget = NetworkTableInstance.getDefault().getTable(tableName).getEntry("tv").getDouble(0.0) == 1;
    SmartDashboard.putNumber("limelightX", horizontalOffset);
    SmartDashboard.putNumber("limelightY", verticalOffset);
    SmartDashboard.putNumber("limelightArea", area);
    SmartDashboard.putBoolean("is limelight detecting target", isPointingAtTarget);
    SmartDashboard.putNumber("PID output", pidOutput);
    pidOutput = autoAim();
    
  }

  

}