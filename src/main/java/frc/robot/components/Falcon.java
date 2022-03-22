// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components;

/** Add your docs here. */
public class Falcon {
    public static double rpmToTicks(double rpm){
        return (rpm * 2048) / 600;
    }
    
    public static double ticksToRPM(double ticks){
        return (ticks * 600) / 2048;
    }

    public static double ticksToDegrees(double ticks) {
        return (ticks / 2048) * 360;
    }

    public static double degreesToTicks(double degrees) {
        return (degrees / 360) * 2048;
    }
}
