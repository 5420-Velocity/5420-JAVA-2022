// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  private WPI_TalonFX liftMotor = new WPI_TalonFX(12);
  private WPI_TalonSRX liftRotationMotor = new WPI_TalonSRX(13);
  private final DigitalInput upperLimit = new DigitalInput(1);
	private final DigitalInput lowerLimit = new DigitalInput(2);

  // This encoder will tell you the position of between the upper and lower encoder
  private final DutyCycleEncoder positioEncoder = new DutyCycleEncoder(3);

  private NetworkTableEntry upper = SmartDashboard.getEntry("upper");
  private NetworkTableEntry lower = SmartDashboard.getEntry("lower");

  public Lift() {
    liftMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void ZeroLiftEncoder(){
    liftMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public void setMotorPower(double power){
    liftMotor.set(power);
  }

  public void setRotationPower(double power){
    liftRotationMotor.set(power);
  }

  public boolean GetUpper(){
    return upperLimit.get();
  }

  public boolean GetLower(){
    return lowerLimit.get();
  }

  public double GetLiftEncoder(){
    return liftMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  @Override
  public void periodic() {
    upper.setBoolean(upperLimit.get());
    lower.setBoolean(lowerLimit.get());
  }
}
