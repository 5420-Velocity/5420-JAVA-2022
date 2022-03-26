// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Lift;

public class NewLiftControl extends CommandBase {
  private Lift lift;
  private Joystick controller;
  private PIDController liftPID = new PIDController(0.1, 0, 0);


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
    double liftAngleInput = -controller.getRawAxis(Constants.ControllerConstants.JOYSTICK_LEFT_X_AXIS);
    double liftInput = controller.getRawAxis(Constants.ControllerConstants.JOYSTICK_RIGHT_Y_AXIS);

    if((liftAngleInput > 0.1 && lift.GetLower()) || (liftAngleInput < -0.1 && lift.GetUpper())){
        this.lift.setRotationPower(liftAngleInput * 0.5);
    }  
    else{
      this.lift.setRotationPower(0);
    }

    

    if(controller.getRawButton(Constants.ControllerConstants.Joystick_Left_Button)){
      this.lift.setMotorPower(liftInput );
    }
    else{
      if(liftInput > 0.1 && this.lift.GetLiftEncoder() < -10){
        this.lift.setMotorPower(liftInput);
      }  
      else if(liftInput < -0.1 && lift.GetLiftEncoder() > -15400){
        double output = Math.abs(liftPID.calculate(lift.GetLiftEncoder(), -15247));
        System.out.println(output);
        this.lift.setMotorPower(liftInput * 0.7);
      }
      else{
        this.lift.setMotorPower(0);
      }
    }

  }

  @Override
  public void end(boolean interrupted) {
    this.lift.setMotorPower(0);
    this.lift.setRotationPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
