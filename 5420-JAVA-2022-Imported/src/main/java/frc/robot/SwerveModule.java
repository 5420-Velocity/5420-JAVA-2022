/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;

  private final CANCoder m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(Constants.DriveTrainConstants.driveSpeedP,
      Constants.DriveTrainConstants.driveSpeedI, Constants.DriveTrainConstants.driveSpeedD);
  private final PIDController m_turningPIDController = new PIDController(Constants.DriveTrainConstants.turnP,
      Constants.DriveTrainConstants.turnI, Constants.DriveTrainConstants.turnD);

  private double m_encoderOffset;

  private boolean _driving;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoder, double encoderOffset) {
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_driveMotor.configFactoryDefault();
    m_driveMotor.setNeutralMode(NeutralMode.Brake);

    m_turningMotor = new WPI_TalonFX(turningMotorChannel);
    m_turningMotor.configFactoryDefault();
    m_turningMotor.setNeutralMode(NeutralMode.Coast);

    m_turningEncoder = new CANCoder(turningEncoder);
    m_encoderOffset = encoderOffset;
    resetDriveEncoder();

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void canDrive(boolean value){
    _driving = value;
  }

  // Do not use unless the turn encoders absolute value changed
  public void setAbsoluteOffset() {
    m_turningEncoder.configMagnetOffset(m_encoderOffset);
  }

  public void resetDriveEncoder() {
    m_driveMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public double GetDriveEncoder() {
    return m_driveMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSensorCollection().getIntegratedSensorVelocity(),
        new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition())));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    // Changing base encoder velocity to meters per socond
    double driveEncoderVelocity = ((m_driveMotor.getSensorCollection().getIntegratedSensorVelocity())
        * Constants.DriveTrainConstants.driveEncoderMultiplier);
    var driveOutput = m_drivePIDController.calculate(driveEncoderVelocity, state.speedMetersPerSecond);

    // Clamps the drive output to prevent damage to gears
    if (driveOutput > Constants.DriveTrainConstants.driveMaxOutput) {
      driveOutput = Constants.DriveTrainConstants.driveMaxOutput;
    } else if (driveOutput < -Constants.DriveTrainConstants.driveMaxOutput) {
      driveOutput = -Constants.DriveTrainConstants.driveMaxOutput;
    }

    // Calculate the turning motor output from the turning PID controller.
    // Changing encoder value to radians between -pi to pi
    double turnEncoderPosition = (m_turningEncoder.getAbsolutePosition()
        * Constants.DriveTrainConstants.turnEncoderMultiplier) - Math.PI;
    var turnOutput = m_turningPIDController.calculate(turnEncoderPosition, state.angle.getRadians());

    // Clamps the turn output to prevent damage to gears
    if (turnOutput > Constants.DriveTrainConstants.turnMaxOutput) {
      turnOutput = Constants.DriveTrainConstants.turnMaxOutput;
    } else if (turnOutput < -Constants.DriveTrainConstants.turnMaxOutput) {
      turnOutput = -Constants.DriveTrainConstants.turnMaxOutput;
    }

    // Set motor power to pid loop outputs
    if(_driving){
      m_driveMotor.set(-driveOutput);
      m_turningMotor.set(turnOutput);
    }
    else{
     m_driveMotor.set(0);
     m_turningMotor.set(0);
    }
  }
}
