/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drivetrain;

public class PixyAlign extends CommandBase {
  
    private Drivetrain driveTrain;
    private AtomicBoolean m_lock;
    private boolean finishable;
    private boolean isFinished;

    private PIDController turnPidController = new PIDController(0.05, 0, 0);

    private NetworkTableEntry hasTarget = SmartDashboard.getEntry("has target");

    public PixyAlign(Drivetrain driveTrain, AtomicBoolean lock, boolean finishable){
        this.driveTrain = driveTrain;
        this.m_lock = lock;
        this.finishable = finishable;
    }

    public PixyAlign(Drivetrain driveTrain){
        this.driveTrain = driveTrain;
        this.m_lock = null;
        this.finishable = true;
    }

    @Override
    public void initialize() {
        this.isFinished = false;
        if(m_lock != null){
            this.m_lock.set(true);
        }
    }

    @Override
     public void execute() {
        if (driveTrain.pixyAlgo.getPixyBest() != null) {
            hasTarget.setBoolean(true);
            // turn to ball
            if(driveTrain.pixyAlgo.getPixyBest().getX() > 140 && driveTrain.pixyAlgo.getPixyBest().getX() < 160){
                this.isFinished = true;
                driveTrain.drive(0, 0, 0, false);
            }
            else{
                double output = turnPidController.calculate(driveTrain.pixyAlgo.getPixyBest().getX(), 150);
                driveTrain.drive(0, 0, output, false);
            }
        }
        else{
            // turn to find ball
            hasTarget.setBoolean(false);
            this.isFinished = true;
        }
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
