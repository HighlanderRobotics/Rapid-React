// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components;

/** Helper class to do unit conversions for Falcon500 motors and encoders/ */
public class Falcon {
    /**Converts an rpm to native falcon500 units. */
    public static double rpmToTicks(double rpm){
        return (rpm * 2048) / 600;
    }
    
    /**Converts native falcon500 units to rpm. */
    public static double ticksToRPM(double ticks){
        return (ticks * 600) / 2048;
    }

    /**Converts native falcon500 units to degrees. */
    public static double ticksToDegrees(double ticks) {
        return (ticks / 2048) * 360;
    }

    /**Converts degrees to native falcon 500 units. */
    public static double degreesToTicks(double degrees) {
        return (degrees / 360) * 2048;
    }
}
