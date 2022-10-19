// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.components.ReversibleDigitalInput;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

/** Contains the hood motor, encoder, and limit switches. */
public class HoodSubsystem extends PIDSubsystem implements Loggable {
    // The motor controller for the hood motor
    public final CANSparkMax hood;
    // The encoder which measures hood extension
    public final Encoder angleEncoder;
    // Limit switches to prevent overrunning endstops
    public final ReversibleDigitalInput topLimitSwitch;
    public final ReversibleDigitalInput bottomLimitSwitch;
    // Current target angle
    private double currentSetpoint;
    
    // Feedforward for the hood
    // Static is set to 0, since I assume we don't need any static power added to the motor
    // Other two numbers were found on https://reca.lc/arm based on the assumptions:
    // 1 NEO 550, 5in to CoM, 5lbs; should update this if we have more information
    public final ArmFeedforward feedforward = new ArmFeedforward(0, 0.008, 0.0391);
    // Total rotations the encoder has gone through
    double rotations;
    
    public HoodSubsystem()
    {
        super(new PIDController(-0.09, 0, 0));
        hood = new CANSparkMax(Constants.HOOD_ANGLE_MOTOR, MotorType.kBrushless);
        hood.setIdleMode(IdleMode.kBrake);
        // Last argument reverses direction
        angleEncoder = new Encoder(Constants.HOOD_ENCODER_A, Constants.HOOD_ENCODER_B, true);
        topLimitSwitch = new ReversibleDigitalInput(Constants.HOOD_LIMIT_SWITCH_TOP, true);
        bottomLimitSwitch = new ReversibleDigitalInput(Constants.HOOD_LIMIT_SWITCH_BOTTOM, true);
    }

    /**Sets the motor power based on PID */
    @Override
    protected void useOutput(double output, double setpoint) {
        // adjust output with feedforward (removed for now)
        double adjustedPower = output + 0; //feedforward.calculate(centerAngle, 0);

        // stop if it tries to go past top limit
        if (topLimitSwitch.get() && adjustedPower < 0) {
            adjustedPower = 0;
        }

        hood.set(adjustedPower);
    }

    // Angle the hood is at when each limit switch is triggered
    //need calibration
    public double topLimit = 40;
    public double bottomLimit = 0;

    /**Sets the current desired angle */
    @Override
    public void setSetpoint(double setpoint) {
        if (setpoint < bottomLimit) {
            setpoint = bottomLimit;
        }
        if(setpoint > topLimit) {
            setpoint = topLimit;
        }
        currentSetpoint = setpoint;
        super.setSetpoint(setpoint);
    }

    /**Gets the current motor positition */
    @Override
    protected double getMeasurement() {
        // 2048 encoder ticks in a rotation
        rotations = angleEncoder.get() / 2048.0;
        //1.47 rotations in the full range - same as around 40 degrees
        return (rotations/1.47) * 40;
    }

    /**Checks if the current angle is close enough to the desired angle */
    @Log
    public boolean atTargetAngle() {
        return Math.abs(currentSetpoint - getMeasurement()) < 5.0;
    }

    /**Gets whether the upper limit switch is pressed */
    public boolean getUpperLimit(){
        return topLimitSwitch.get();
    }
    
    /**Gets whether the lower limit switch is pressed */
    public boolean getLowerLimit(){
        return bottomLimitSwitch.get();
    }

    /**Runs periodically. Resets the hood angle when the lower limit switch is pressed and updates the PID */
    @Override
    public void periodic() {
        super.periodic();
        if (topLimitSwitch.get()) {
            // don't want this to mess up the top limit; what should we do when the top limit is hit?
            //topLimit = angleEncoder.getDistance();
            
        }
        if (bottomLimitSwitch.get()){
            // bottom position should be 0
           angleEncoder.reset();
        }

    }
}
