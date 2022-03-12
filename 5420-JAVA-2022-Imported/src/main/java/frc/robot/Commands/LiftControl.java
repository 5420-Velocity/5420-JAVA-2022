// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.*;

public class LiftControl extends CommandBase {
  private Lift lift;
  private double power;


  public LiftControl(Lift lift, Double power) {
    this.lift = lift;
    this.power = power;
  }

  @Override
  public void initialize() {
 
  }

  @Override
  public void execute() {
    if(power > 0){
      if(!lift.GetUpper()){
        lift.setMotorPower(0); 
      }
      else{
        System.out.println(power);
        lift.setMotorPower(power);
      }
    }
    else{
      if(!lift.GetLower()){
        lift.setMotorPower(0);
      }
      else{
        lift.setMotorPower(power);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    lift.setMotorPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
