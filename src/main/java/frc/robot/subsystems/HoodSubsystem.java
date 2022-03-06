// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.components.LimitSwitch;
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
    public final LimitSwitch topLimitSwitch;
    public final LimitSwitch bottomLimitSwitch;
    
    // Feedforward for the hood
    // Static is set to 0, since I assume we don't need any static power added to the motor
    // Other two numbers were found on https://reca.lc/arm based on the assumptions:
    // 1 NEO 550, 5in to CoM, 5lbs; should update this if we have more information
    public final ArmFeedforward feedforward = new ArmFeedforward(0, 0.008, 0.0391);
    @Log
    double rotations;
    
    public HoodSubsystem()
    {
        super(new PIDController(-0.09, 0, 0));
        hood = new CANSparkMax(Constants.HOOD_ANGLE_MOTOR, MotorType.kBrushless);
        hood.setIdleMode(IdleMode.kBrake);
        // Last argument reverses direction
        angleEncoder = new Encoder(Constants.HOOD_ENCODER_A, Constants.HOOD_ENCODER_B, true);
        topLimitSwitch = new LimitSwitch(Constants.HOOD_LIMIT_SWITCH_TOP, true);
        bottomLimitSwitch = new LimitSwitch(Constants.HOOD_LIMIT_SWITCH_BOTTOM, true);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // the angle to the center of mass
        // assuming this is 5 degrees from the center
        double centerAngle = Math.toRadians(setpoint - 5);
        // adjust output with feedforward (removed for now)
        double adjustedPower = output + 0; //feedforward.calculate(centerAngle, 0);

        // stop if it tries to go past top limit
        if (topLimitSwitch.get() && adjustedPower < 0) {
            adjustedPower = 0;
        }

        hood.set(adjustedPower);
        //System.out.println(adjustedPower);
    }

    //need calibration
    public double topLimit = 40;
    public double bottomLimit = 0;

    @Override
    @Config
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
        //1.47 rotations in the full range - same as around 40 degrees
        return (rotations/1.47) * 40;
    }
    @Log
    public boolean getUpperLimit(){
        return topLimitSwitch.get();
    }
    @Log
    public boolean getLowerLimit(){
        return bottomLimitSwitch.get();
    }

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
