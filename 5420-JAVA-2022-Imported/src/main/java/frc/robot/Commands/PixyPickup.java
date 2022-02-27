/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.*;

public class PixyPickup extends CommandBase {
  
    private Drivetrain driveTrain;
    private Intake intake;
    private double power;
    private boolean isFinished;

    private PIDController turnPidController = new PIDController(0.025, 0, 0);

    private NetworkTableEntry hasTarget = SmartDashboard.getEntry("has target");

    public PixyPickup(Drivetrain driveTrain, Intake intake, double power){
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.power = power;
    }

    @Override
    public void initialize() {
        this.isFinished = false;
    }

    @Override
     public void execute() {
    
        if (driveTrain.pixyAlgo.getPixyBest() != null) {
            hasTarget.setBoolean(true);
            
            if((driveTrain.pixyAlgo.getPixyBest().getX() < 145 || driveTrain.pixyAlgo.getPixyBest().getX() > 155) || driveTrain.pixyAlgo.getPixyBest().getArea() <= Constants.DriveTrainConstants.pixyTargetArea){
              // turn to ball
              double output = turnPidController.calculate(driveTrain.pixyAlgo.getPixyBest().getX(), 150);
              driveTrain.CanDrive(true);
              driveTrain.drive(power, 0, output, false);
              intake.setIntakePower(-0.5);
              System.out.println(driveTrain.pixyAlgo.getPixyBest().getSignature());
            }
            else{
              driveTrain.CanDrive(false);
              intake.setIntakePower(0);
              driveTrain.drive(0, 0, 0, false);
              this.isFinished = true;
            }
        }
        else{
          hasTarget.setBoolean(false);
            this.isFinished = true;
        }
    }

    public static double getCurve(double input) {
        double sign = Math.signum(input);
    
        double value = Math.abs(input);
        value = Math.pow(value, 2);
        value += 0.02;
    
        return sign * value;
    }
    
    @Override
    public void end(boolean interrupted) {
      driveTrain.CanDrive(false);
      intake.setIntakePower(0);
    }

    @Override
    public boolean isFinished() {
      return this.isFinished;
    }
}
