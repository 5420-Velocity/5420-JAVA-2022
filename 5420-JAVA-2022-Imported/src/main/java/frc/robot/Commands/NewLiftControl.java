// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Lift;

public class NewLiftControl extends CommandBase {
  private Lift lift;
  private Joystick controller;
  private PIDController liftPID = new PIDController(0.4, 0, 0);


  public NewLiftControl(Lift lift, Joystick controller) {
    this.lift = lift;
    this.controller = controller;
    addRequirements(lift);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(controller.getRawAxis(Constants.ControllerConstants.JOYSTICK_LEFT_X_AXIS) > 0.1 && lift.GetLower()){
        this.lift.setRotationPower(controller.getRawAxis(Constants.ControllerConstants.JOYSTICK_LEFT_X_AXIS) * 0.8);
    }  
    else if(controller.getRawAxis(Constants.ControllerConstants.JOYSTICK_LEFT_X_AXIS) < -0.1 && lift.GetUpper()){
        this.lift.setRotationPower(controller.getRawAxis(Constants.ControllerConstants.JOYSTICK_LEFT_X_AXIS) * 0.8);
    }
    else{
      this.lift.setRotationPower(0);
    }

    if(controller.getRawAxis(Constants.ControllerConstants.JOYSTICK_LEFT_Y_AXIS) > 0.1){
      // System.out.println("positive");
      // double output = liftPID.calculate(lift.GetLiftEncoder(), 0.5);
      // if(lift.GetLiftEncoder() > 0.5){
      //   output = Math.abs(output);
      // }
      // System.out.println(output);
      this.lift.setMotorPower(controller.getRawAxis(Constants.ControllerConstants.JOYSTICK_LEFT_Y_AXIS));
    }  
    else if(controller.getRawAxis(Constants.ControllerConstants.JOYSTICK_LEFT_Y_AXIS) < -0.1){
      double output = liftPID.calculate(lift.GetLiftEncoder(), 8.5);
      this.lift.setMotorPower(controller.getRawAxis(Constants.ControllerConstants.JOYSTICK_LEFT_Y_AXIS));
    }
    else{
      this.lift.setMotorPower(0);
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
