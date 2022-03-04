/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.LinkedList;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FeedMoment;
import frc.robot.FeedMoment.MomentType;
import frc.robot.Subsystems.*;

public class AutoShoot extends CommandBase {

	private final Shooter _shooter;
	private final LimeLight _limelight;
	private Date speedRampUpTime;
	private LinkedList<FeedMoment> shootInterval = new LinkedList<FeedMoment>();
	private FeedMoment currentDeadline;
	private boolean isFinished = false;
	private double power;
	private int ballCount;

	public AutoShoot(Shooter newShooter, double value, int balls){
		this._shooter = newShooter;
		this.power = value;
		this._limelight = null;
		this.ballCount = balls;
	}

	public AutoShoot(Shooter newShooter, LimeLight limelight, int balls){
		this._limelight = limelight;
		this._shooter = newShooter;
		this.ballCount = balls;
	}
	

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

		this.isFinished = false;
		this.currentDeadline = null;
		int rampUpTime;

		if(power < 0.6){
			rampUpTime = 600;
		}
		else{
			rampUpTime = 1200; // Delay Time norm 1800 but I'm getting rid of it ~Jake
		}
		int feedForwardTime = 1200; // Forward Feed Time
		int feedReverseTime = 300; // Reverse Feed Time
		int feedTimeSpace = 1500; // Off Time

		Calendar calculateDate = GregorianCalendar.getInstance();
		calculateDate.add(GregorianCalendar.MILLISECOND, rampUpTime);
		this.speedRampUpTime = calculateDate.getTime();

		int timelinePosition = rampUpTime;

		// This is needed to set the correct interval.
		Calendar sacrificialInit = GregorianCalendar.getInstance();
		sacrificialInit.add(GregorianCalendar.MILLISECOND, timelinePosition);
		this.shootInterval.add(new FeedMoment(MomentType.Off, sacrificialInit.getTime()));

		// Store the times of when to trigger the shooting
		for (int i = 0; i < ballCount; i++) {
			// Forward
			Calendar calculateDateBallOn = GregorianCalendar.getInstance();
			timelinePosition += feedForwardTime;
			calculateDateBallOn.add(GregorianCalendar.MILLISECOND, timelinePosition);
			this.shootInterval.add(new FeedMoment(MomentType.Forward, calculateDateBallOn.getTime()));

			// Reverse
			Calendar calculateDateBallFeedReverse = GregorianCalendar.getInstance();
			timelinePosition += feedReverseTime;
			calculateDateBallFeedReverse.add(GregorianCalendar.MILLISECOND, timelinePosition);
			//this.shootInterval.add(new FeedMoment(MomentType.Reverse, calculateDateBallFeedReverse.getTime()));

			// Off
			Calendar calculateDateBallFeedOff = GregorianCalendar.getInstance();
			timelinePosition += feedTimeSpace;
			calculateDateBallFeedOff.add(GregorianCalendar.MILLISECOND, timelinePosition);
			this.shootInterval.add(new FeedMoment(MomentType.Off, calculateDateBallFeedOff.getTime()));
		}
		if(_limelight != null){
			_limelight.setLedMode(0);
		}
		
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(_limelight == null){
			this._shooter.setShooterPower(power);
		}
		else{
			this._shooter.setShooterPower(_limelight.getShooterPower());
		}

		if (this.speedRampUpTime != null) {
			this._shooter.setFeedPower(0);
			// We can run the speed ramp.
			if (new Date().after(this.speedRampUpTime)) {
				this.speedRampUpTime = null;
			}
		}
		else {
			// Speed Ramp Complete, Fire using the feeder
			if (this.currentDeadline == null) {
				// If we dont have a Deadline.
				if (this.shootInterval.size() == 0) {
					// We are out of tasks the Routine is Complete, Mark Command is finished
					this.isFinished = true;
				}
				else {
					// Grab the next Deadline
					this.currentDeadline = this.shootInterval.pop();
				}
			}

			if (this.currentDeadline != null) {
				// We have a deadline

				if (this.currentDeadline.expired()) {
					// If we have a deadline AND it's expired
					this.currentDeadline = null;
				}
				else {
					// If we have a deadline
					if (this.currentDeadline.type == MomentType.Off) {
						this._shooter.setFeedPower(0);
					}
					else if (this.currentDeadline.type == MomentType.Forward) {
						this._shooter.setFeedPower(-0.5);
					}
					else if (this.currentDeadline.type == MomentType.Reverse) {
						this._shooter.setFeedPower(-0.4);
					}
				}
			}

		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if(_limelight != null){
			_limelight.setLedMode(1);
		}
		this._shooter.setFeedPower(0);
		this._shooter.setShooterPower(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return this.isFinished;
	}
}
