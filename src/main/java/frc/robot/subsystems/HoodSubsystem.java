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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class HoodSubsystem extends PIDSubsystem implements Loggable {
  /** Creates a new ExampleSubsystem. */
  public final CANSparkMax hood;
   
public HoodSubsystem()
{
    super(new PIDController(0.5, 0, 0));
    hood = new CANSparkMax(Constants.HOOD_ANGLE_MOTOR, MotorType.kBrushless);
    hood.setIdleMode(IdleMode.kBrake);
}



@Override
protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub
    hood.set(output);
    //System.out.println(output);
}
@Override
@Config
public void setSetpoint(double setpoint){
    if (setpoint <13)
    {
         setpoint = 13;
    }
    if(setpoint>46)
    {
        setpoint = 46;
    }
    super.setSetpoint(setpoint);
}
@Override
@Log
protected double getMeasurement() {
    // TODO Auto-generated method stub
     double rotations = hood.getEncoder().getPosition();
     //this is to measure the angle and we set it to 13, the starting angle. 20:1 is the gear ratio. 
    return rotations/20*360 + 13;
    
    

}

  
}
