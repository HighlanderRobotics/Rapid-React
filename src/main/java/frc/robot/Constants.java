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

    public static final PIDController AUTOAIM_PID_CONTROLLER = new PIDController(0.3, 0, 0);
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.44; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.44; // FIXME Measure and set wheelbase

    public static final double ROTATION_SPEED_MULTPILIER = 0.4;
    public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 6; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(134 + 180); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 1; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 20; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(129 + 180); // FIXME Measure and set back left steer offset

    public static final int INNER_FEEDER_MOTOR = 9;
    public static final int OUTER_FEEDER_MOTOR = 12;
    public static final int FLYWHEEL_MOTOR = 10;
    public static final int HOOD_ANGLE_MOTOR = 11;

    public static final int HOOD_ENCODER_A = 7; // dio ports for A and B for quadrature on rev encoder for hood
    public static final int HOOD_ENCODER_B = 8;

    public static final int HOOD_LIMIT_SWITCH_TOP = 3;
    public static final int HOOD_LIMIT_SWITCH_BOTTOM = 2;
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(107 + 180); // FIXME Measure and set back right steer offset
    public static final int INTAKE_MOTOR = 18;
    public static final int INTAKE_SOLENOID_FORWARD = 0;
    public static final int INTAKE_SOLENOID_BACKWARD = 1;
    public static final int LOWER_BEAMBREAK = 4;
    public static final int UPPER_BEAMBREAK = 5;

    public static final int CLIMBER_ANGLE_MOTOR = 16;
    public static final int CLIMBER_EXTENSION_MOTOR = 15;
    public static final int CLIMBER_LIMIT_SWITCH = 1;
    public static final int CLIMBER_RATCHET_SERVO = 9;
}
