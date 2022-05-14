// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
  private WPI_TalonSRX liftMotor = new WPI_TalonSRX(12); //was ID55
  // private WPI_TalonSRX liftMotor2 = new WPI_TalonSRX(55);
  
  //current sensor
  public final AnalogInput LiftCurrentSensor = new AnalogInput(Constants.DriveTrainConstants.LiftCurrentSensor);
  private final NetworkTableEntry LiftCurrentSensorNT = SmartDashboard.getEntry(Constants.DriveTrainConstants.LiftCurrentSensorNT);
  private final NetworkTableEntry LiftCurrentSensorNTVolts = SmartDashboard.getEntry(Constants.DriveTrainConstants.LiftCurrentSensorNT + " Volts");

  //rigid hook limit switch
  public final AnalogInput RigidHookLimitSwitch = new AnalogInput(Constants.DriveTrainConstants.RigidHookLimitSwitch);
  public final NetworkTableEntry RigidHookLimitSwitchNT = SmartDashboard.getEntry(Constants.DriveTrainConstants.RigidHookLimitSwitchNT);

  // This encoder will tell you the position of between the upper and lower encoder
  private final Encoder positionEncoder = new Encoder(3, 4);
  private NetworkTableEntry liftEncoder = SmartDashboard.getEntry("Lift Encoder");

  public Lift() {
    liftMotor.setNeutralMode(NeutralMode.Brake);
    // liftMotor2.setNeutralMode(NeutralMode.Brake);
  }

  public void ZeroLiftEncoder(){
    positionEncoder.reset();
  }

  public void setMotorPower(double power){
    liftMotor.set(power);
    // liftMotor2.set(power);
  }

  public double GetLiftEncoder(){
    return positionEncoder.get();
  }

  @Override
  public void periodic() {
    liftEncoder.setDouble(GetLiftEncoder());
    
    LiftCurrentSensorNT.setNumber(LiftCurrentSensor.getAverageValue());
    LiftCurrentSensorNTVolts.setNumber(LiftCurrentSensor.getAverageVoltage());
  }
}
