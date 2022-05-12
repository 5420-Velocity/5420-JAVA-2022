package frc.robot.Commands.newAutoStuff;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Lift;
import frc.robot.utils.StateList;

public class RetractWithCurrentSensor extends CommandBase {
    private Lift lift;
    
    private StateList stateList;

    public static final int RETRACT_ENCODER_LIMIT_PARTIAL = -2000;
    private static final double RETRACT_POWERLIMIT = 0.9;

    
    public RetractWithCurrentSensor(Lift lift) {
        this.lift = lift;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        double output = this.lift.LiftCurrentSensor.getAverageValue();
                 
 
        if (Math.abs(output) > 520){
            this.lift.setMotorPower(RETRACT_POWERLIMIT);
        }
    }
    @Override
    public void end(boolean interrupted) {
        // When the comammand has been ended OR was canceled, stop the motor
        this.lift.setMotorPower(0);
        System.out.println("Finsihed " + this.getName());
    }
    

    @Override
    public boolean isFinished() {
        return this.lift.GetLiftEncoder() < RETRACT_ENCODER_LIMIT_PARTIAL;
        
    }
}



