// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  private WPI_TalonFX liftMotor = new WPI_TalonFX(12);
  public Lift() {
    liftMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void ZeroLiftEncoder(){
    liftMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public void setMotorPower(double power){
    liftMotor.set(power);
  }

  public double GetLiftEncoder(){
    return liftMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
