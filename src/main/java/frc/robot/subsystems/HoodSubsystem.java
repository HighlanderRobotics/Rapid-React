// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class HoodSubsystem extends PIDSubsystem implements Loggable {
    /** Creates a new ExampleSubsystem. */
    public final CANSparkMax hood;
    public final Encoder angleEncoder;
    public final DigitalInput topLimitSwitch;
    public final DigitalInput bottomLimitSwitch;
    //need calibration
    public double topLimit = 10;
    public double bottomLimit = 0;
    // Feedforward for the hood
    // Static is set to 0, since I assume we don't need any static power added to the motor
    // Other two numbers were found on https://reca.lc/arm based on the assumptions:
    // 1 NEO 550, 5in to CoM, 5lbs; should update this if we have more information
    public final ArmFeedforward feedforward = new ArmFeedforward(0, 0.08, 3.91);
    @Log
    double rotations;
    
    public HoodSubsystem()
    {
        super(new PIDController(-0.03, 0, 0));
        hood = new CANSparkMax(Constants.HOOD_ANGLE_MOTOR, MotorType.kBrushless);
        hood.setIdleMode(IdleMode.kBrake);
        // Last argument reverses direction
        angleEncoder = new Encoder(Constants.HOOD_ENCODER_A, Constants.HOOD_ENCODER_B, true);
        topLimitSwitch = new DigitalInput(Constants.HOOD_LIMIT_SWITCH_TOP);
        bottomLimitSwitch = new DigitalInput(Constants.HOOD_LIMIT_SWITCH_BOTTOM);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // the angle to the center of mass
        // assuming this is 5 degrees from the center
        double centerAngle = Math.toRadians(setpoint - 5);
        // setting velocity to 0 since we want it to stop?
        double feed = feedforward.calculate(centerAngle, 0);
        hood.set(output + feed);
        System.out.println(output + ", " + feed);
    }

    @Override
    public void setSetpoint(double setpoint) {
        if (setpoint < bottomLimit) {
            setpoint = bottomLimit;
        }
        if(setpoint > topLimit) {
            setpoint = topLimit;
        }
        super.setSetpoint(setpoint);
    }

    @Override
    @Log
    protected double getMeasurement() {
        // 2048 encoder ticks in a rotation
        rotations = angleEncoder.get() / 2048.0;
        // measured angles: min is 24 max 58
        // this is to measure the angle and we set it to 13, the starting angle. 20:1 is the (probably wrong?) gear ratio. 
        return rotations/20 * 360 + 13;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (topLimitSwitch.get()) {
            topLimit = angleEncoder.getDistance();
        }
        if (bottomLimitSwitch.get()){
            bottomLimit = angleEncoder.getDistance();
        }
    }
}
