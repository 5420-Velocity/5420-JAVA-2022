// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class AutoPixyAlign extends CommandBase {
  
    private Drivetrain driveTrain;
    private AtomicBoolean m_lock;
    private Joystick m_joystick;
    private boolean finishable;
    private boolean isFinished;

    private PIDController turnPidController = new PIDController(0.03, 0, 0);

    private NetworkTableEntry hasTarget = SmartDashboard.getEntry("has target");

    private int x;
    private int y;
    private int r;


    @Override
    public void initialize() {
        this.isFinished = false;
        if(m_lock != null){
            this.m_lock.set(true);
        }
    }

    @Override
     public void execute() {

        final var xSpeed = -m_joystick.getRawAxis(x) * Constants.DriveTrainConstants.kMaxSpeed;
        // Determine rotation speed for y-axis.
        final var ySpeed = -m_joystick.getRawAxis(y) * Constants.DriveTrainConstants.kMaxSpeed;
        final var rotSpeed = -m_joystick.getRawAxis(r) * Constants.DriveTrainConstants.kMaxSpeed;
    
        if (driveTrain.pixyAlgo.getPixyBest() != null) {
            hasTarget.setBoolean(true);
            // turn to ball
            double output = turnPidController.calculate(driveTrain.pixyAlgo.getPixyBest().getX(), 150);
            driveTrain.CanDrive(true);
            driveTrain.drive(getCurve(xSpeed), getCurve(ySpeed), output, false);
        }
        else{
            // turn to find ball
            hasTarget.setBoolean(false);

            if ((Math.abs(xSpeed) < Constants.ControllerConstants.NoInputTolerance)
                    && (Math.abs(ySpeed) < Constants.ControllerConstants.NoInputTolerance)
                    && (Math.abs(rotSpeed) < Constants.ControllerConstants.NoInputTolerance)) {
                driveTrain.CanDrive(false);
            }
            else{
                driveTrain.CanDrive(true);
                driveTrain.drive(getCurve(xSpeed), getCurve(ySpeed), getCurve(rotSpeed), driveTrain.IsFieldRelative());
            }
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
        if(m_lock != null){
            this.m_lock.set(false);
        }
    }

    @Override
    public boolean isFinished() {
        if(finishable){
            return this.isFinished;
        }
        else{
            return false;
        }
    }
}
