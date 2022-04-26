// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Lift;

public class AutoClimb extends CommandBase {
  private Lift lift;
  private Boolean isFinished;
  public Boolean isHooked;
  private int output;
  private boolean interrupted;


  public AutoClimb(Lift lift, boolean isHooked) {
    this.lift = lift;
    this.isHooked = new boolean;
    int a = 1;
    int b = 0;
    boolean a1 = true;
    boolean b1 = false;
    //do i need this
    addRequirements(lift);
    // implementing limit switch 
    //fix boolean, code will only work if there's more than one limit switch?
    DigitalInput stationaryHookSwitch = new DigitalInput(1);
    if (stationaryHookSwitch.get()){
        this.isHooked = true;
        output = Math.min(output, 0);
    }
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //do i need this
    this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //implement current sensor
      //first part, starting on first bar
    if (this.isHooked = true){
      System.out.println("going to next bar");
      //fix sequential command so it doesn't end with if statement
      new SequentialCommandGroup(
        this.lift.setMotorPower(1));
        //clean this code up
        if (this.lift.GetLiftEncoder() ==-16050){
          this.lift.setMotorPower(0);
        this.lift.setRotationPower(1);
        this.lift.setMotorPower(-1);
        if (this.lift.GetLiftEncoder() == 5){
          this.lift.setMotorPower(0);
      end(interrupted);
    }
    //second part, pattern begins
    if (this.isHooked = true){
      System.out.println("going to next bar");
      //fix sequential command so it doesn't end with if statement
      new SequentialCommandGroup(
        this.lift.setMotorPower(1));
        //clean this code up
        if (this.lift.GetLiftEncoder() ==-16050){
          this.lift.setMotorPower(0);
        this.lift.setRotationPower(1);
        this.lift.setRotationPower(-1);
        if (this.lift.GetLiftEncoder() == 5){
          this.lift.setMotorPower(0);
        }
      end(interrupted);
    }
  }
}
  }


    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //set lift power to 0
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
