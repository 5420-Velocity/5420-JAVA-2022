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
import frc.robot.Subsystems.LiftRotationMechanism;

public class NewLiftControl extends CommandBase {
  private Lift lift;
  private Joystick controller;
  private PIDController liftPID = new PIDController(0.1, 0, 0);
  private LiftRotationMechanism LiftRotationMechanism;

  private static final int EXTEND_ENCODER_LIMIT = -16050;
  private static final double RETRACT_POWERLIMIT = 0.9;
  private static final double EXTEND_POWERLIMIT = 0.7;


  public NewLiftControl(Lift lift, LiftRotationMechanism liftRotationMechanism, Joystick controller) {
    this.lift = lift;
    this.controller = controller;
    this.LiftRotationMechanism = liftRotationMechanism;
    addRequirements(lift);
    addRequirements(liftRotationMechanism);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double liftAngleInput = controller.getRawAxis(Constants.ControllerConstants.JOYSTICK_RIGHT_X_AXIS);
    double liftInput = controller.getRawAxis(Constants.ControllerConstants.JOYSTICK_RIGHT_Y_AXIS);

    if((liftAngleInput > 0.1 && LiftRotationMechanism.GetLower()) || (liftAngleInput < -0.1 && LiftRotationMechanism.GetUpper())){
        this.LiftRotationMechanism.setRotationPower(liftAngleInput * 0.5);
    }  
    else{
      this.LiftRotationMechanism.setRotationPower(0);
    }

    if(controller.getRawButton(Constants.ControllerConstants.Joystick_Left_Button)){
      this.lift.setMotorPower(liftInput );
    }
    else{
      if(liftInput > 0.1 && this.lift.GetLiftEncoder() < 50){
        this.lift.setMotorPower(liftInput * RETRACT_POWERLIMIT);
      } 
      else if(liftInput < -0.1 && lift.GetLiftEncoder() > EXTEND_ENCODER_LIMIT){
        double output = Math.abs(liftPID.calculate(lift.GetLiftEncoder(), EXTEND_ENCODER_LIMIT));
        System.out.println(output);
        this.lift.setMotorPower(liftInput * EXTEND_POWERLIMIT);
      }
      else{
        this.lift.setMotorPower(0);
      }
    }

  }

  @Override
  public void end(boolean interrupted) {
    this.lift.setMotorPower(0);
    this.LiftRotationMechanism.setRotationPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
