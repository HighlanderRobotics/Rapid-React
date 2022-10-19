// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final PIDController AUTOAIM_PID_CONTROLLER = new PIDController(0.7, 0.0, -0.01); //(0.004, 0.0, -0.00004)
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.44; 
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.44; 

    public static final double ROTATION_SPEED_MULTPILIER = 0.55;
    public static final int DRIVETRAIN_PIGEON_ID = 0; 

    // Swerve motor CAN ids and encoder offsets

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 6; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(134 + 180); // was 134 + 180

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0); // was 0

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 20; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(129 + 180); 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(107 + 180); 

    // Routing modor CAN ids
    public static final int INNER_FEEDER_MOTOR = 9;
    public static final int OUTER_FEEDER_MOTOR = 12;
    // Routing beambreak ports
    public static final int LOWER_BEAMBREAK = 4;
    public static final int UPPER_BEAMBREAK = 5;

    // Flywheel and hood CAN ids
    public static final int FLYWHEEL_MOTOR = 10;
    public static final int HOOD_ANGLE_MOTOR = 11;

    // DIO ports for A and B for quadrature on rev encoder for hood
    public static final int HOOD_ENCODER_A = 7; 
    public static final int HOOD_ENCODER_B = 8;

    // DIO ports for the hood limit switches
    public static final int HOOD_LIMIT_SWITCH_TOP = 3;
    public static final int HOOD_LIMIT_SWITCH_BOTTOM = 2;
    
    // Intake motor CAN id
    public static final int INTAKE_MOTOR = 18;
    // Intake solenoid port
    public static final int INTAKE_SOLENOID_FORWARD = 0;
    public static final int INTAKE_SOLENOID_BACKWARD = 1;
    
    // Climber motor CAN ids. Angle motor is not used on the new climber
    public static final int CLIMBER_ANGLE_MOTOR = 16;
    public static final int CLIMBER_EXTENSION_MOTOR = 15;
    public static final int CLIMBER_LIMIT_SWITCH = 1;
    public static final int CLIMBER_RATCHET_SERVO = 9;
    // Solenoids for releasing the passive arms on the new climber
    public static final int CLIMBER_SOLENOID_FORWARD = 2;
    public static final int CLIMBER_SOLENOID_BACKWARD = 3;
    
    // Port for the all-important LEDS
    public static final int LED_PORT = 8;
}
