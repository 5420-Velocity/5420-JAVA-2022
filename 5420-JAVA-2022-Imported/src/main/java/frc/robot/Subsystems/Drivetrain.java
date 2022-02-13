/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import java.lang.Math;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveTrainConstants;
import io.github.pseudoresonance.pixy2api.Pixy2;
import frc.robot.Constants;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Represents a swerve drive style drivetrain.
 */
public class Drivetrain extends SubsystemBase {

  private final Translation2d m_frontLeftLocation = new Translation2d(0.3, 0.3);
  private final Translation2d m_frontRightLocation = new Translation2d(0.3, -0.3);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.3, 0.3);
  private final Translation2d m_backRightLocation = new Translation2d(-0.3, -0.3);

  private final SwerveModule m_frontLeft = new SwerveModule(DriveTrainConstants.frontLeftDrive, DriveTrainConstants.frontLeftTurn,
   DriveTrainConstants.frontLeftEncoder, 310.43);
  private final SwerveModule m_frontRight = new SwerveModule(DriveTrainConstants.frontRightDrive, DriveTrainConstants.frontRightTurn,
   DriveTrainConstants.frontRightEncoder, 157.5);
  private final SwerveModule m_backLeft = new SwerveModule(DriveTrainConstants.backLeftDrive, DriveTrainConstants.backLeftTurn,
   DriveTrainConstants.backLeftEncoder, 97.119);
  private final SwerveModule m_backRight = new SwerveModule(DriveTrainConstants.backRightDrive, DriveTrainConstants.backRightTurn,
   DriveTrainConstants.backRightEncoder, 88.77);

  private final PigeonIMU m_gyro = new PigeonIMU(DriveTrainConstants.pigeon);

  private boolean isFieldRelative;
  private boolean isXDefault;

  private double MaxSpeed = Constants.DriveTrainConstants.kMaxSpeed;
  
  private int signature;
  private NetworkTableEntry isRed = SmartDashboard.getEntry("Red");

  private final Pixy2 pixy = Pixy2.createInstance(DriveTrainConstants.pixyLink);
  public final PixyAlgo pixyAlgo = new PixyAlgo(pixy);

  public enum module {
    frontLeft, frontRight, backLeft, backRight;
  }

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
      m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());


  public Drivetrain(){
    isRed.setBoolean(false);
    	// Tries to Communicate with the Pixy at this moment
		this.pixy.init(DriveTrainConstants.pixyLinkPort.value);
		this.pixy.setLamp((byte) 1, (byte) 1);
    this.pixy.setLED(129, 183, 219);
    signature = 2;
  }

  @Override
	public void periodic() {
    if(isRed.getBoolean(false)){
      signature = 1;
    }
    else{
      signature = 2;
    }
    this.pixy.getCCC().getBlocks(false, signature, 4);
  }

  public void setModule(double power){
    m_backRight.setDesiredState(new SwerveModuleState(1, new Rotation2d(1)));
  }
  
  public void CanDrive(boolean value){
    m_frontLeft.canDrive(value);
    m_frontRight.canDrive(value);
    m_backLeft.canDrive(value);
    m_backRight.canDrive(value);
  }

  // Returns the angle of the robot as a Rotation2d.
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_gyro.getFusedHeading());
  }

  // used to zero the gyro at competition when the robot is lined up with the
  // field
  public void zeroGyroHeading() {
    m_gyro.setFusedHeading(0.0);
  }

  public void setGyro(double degrees) {
    m_gyro.setFusedHeading(degrees);
  }

  public boolean IsFieldRelative() {
    return isFieldRelative;
  }

  public void SetFieldRelative(boolean newValue) {
    this.isFieldRelative = newValue;
  }

  public boolean isXDefault(){
    return isXDefault;
  }

  public void setXDefault(boolean value){
    isXDefault = value;
  }

  public void StopModules(){
    m_frontLeft.StopMotors();
    m_frontRight.StopMotors();
    m_backLeft.StopMotors();
    m_backRight.StopMotors();
  }

  public void setMaxSpeed(double value) {
    MaxSpeed = Constants.DriveTrainConstants.kMaxSpeed + value;
  }

  public double getMaxSpeed() {
    return MaxSpeed;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics
        .toSwerveModuleStates(fieldRelative ? fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  private ChassisSpeeds fromFieldRelativeSpeeds(double vxMetersPerSecond, double vyMetersPerSecond,
      double omegaRadiansPerSecond, Rotation2d robotAngle) {
    return new ChassisSpeeds(vxMetersPerSecond * robotAngle.getCos() - vyMetersPerSecond * robotAngle.getSin(),
        vxMetersPerSecond * robotAngle.getSin() + vyMetersPerSecond * robotAngle.getCos(), omegaRadiansPerSecond);
  }

  public void setWheelAngleStates(double fl, double fr, double bl, double br) {
    setWheelState(module.frontLeft, fl, 0.0);
    setWheelState(module.frontRight, fr, 0.0);
    setWheelState(module.backLeft, bl, 0.0);
    setWheelState(module.backRight, br, 0.0);
  }

  public double[] getWheelAngles(){
    double[] wheelAngles = new double[] {getWheelState(module.frontLeft).angle.getDegrees(), getWheelState(module.frontRight).angle.getDegrees(),
      getWheelState(module.backLeft).angle.getDegrees(), getWheelState(module.backRight).angle.getDegrees()};
    return wheelAngles;
  }

  public void setWheelState(module module, double angle, double speed) {
    angle = Math.toRadians(angle);

    switch (module) {
    case frontLeft:
      m_frontLeft.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
      break;
    case frontRight:
      m_frontRight.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
      break;
    case backLeft:
      m_backLeft.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
      break;
    case backRight:
      m_backRight.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
      break;
    }
  }

  public double getWheelDriveEncoder(module module) {
    switch (module) {
    case frontLeft:
      return m_frontLeft.GetDriveEncoder();
    case frontRight:
      return m_frontRight.GetDriveEncoder();
    case backLeft:
      return m_backLeft.GetDriveEncoder();
    case backRight:
      return m_backRight.GetDriveEncoder();
    default:
      return 0;
    }
  }

  public SwerveModuleState getWheelState(module module) {
    switch (module) {
    case frontLeft:
      return m_frontLeft.getState();
    case frontRight:
      return m_frontRight.getState();
    case backLeft:
      return m_backLeft.getState();
    case backRight:
      return m_backRight.getState();
    default:
      return null;
    }
  }

  public void resetAllDriveEncoders() {
    m_frontLeft.resetDriveEncoder();
    m_frontRight.resetDriveEncoder();
    m_backLeft.resetDriveEncoder();
    m_backRight.resetDriveEncoder();
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_odometry.update(getAngle(), m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
        m_backRight.getState());
  }
}
