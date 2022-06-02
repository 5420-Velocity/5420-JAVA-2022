package frc.robot.Commands.newAutoStuff;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.LiftRotationMechanism;

public class RotateArmBackPartial extends CommandBase{
    private LiftRotationMechanism liftRotationMechanism;
    private double liftAngleInput = 0.5;
    
    private Date endTime;

    public RotateArmBackPartial(LiftRotationMechanism liftRotationMechanism) {
        this.liftRotationMechanism = liftRotationMechanism;
    }

    public void execute(){
        // Set the motor speed if the Upper Limit (invered) is detected
        this.liftRotationMechanism.setRotationPower(liftAngleInput);
    }

    @Override
    public void end(boolean interrupted) {
        this.liftRotationMechanism.setRotationPower(0);
        System.out.println("Finsihed " + this.getName());
    }


    public boolean isFinished() {
        if (!liftRotationMechanism.GetUpper()) {
            // The sensor has been reached.
            // Set the end time
            // Read the end time if we have passed
            if (this.endTime == null) {
                Calendar calculateDate = GregorianCalendar.getInstance();
                calculateDate.add(GregorianCalendar.MILLISECOND, 500);
                this.endTime = calculateDate.getTime();
            }
            
            return new Date().after(endTime);
        } else {
            this.endTime = null;
        }
        return false;
    }
}
