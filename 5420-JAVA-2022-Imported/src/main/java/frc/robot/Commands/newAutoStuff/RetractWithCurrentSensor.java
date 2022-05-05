package frc.robot.Commands.newAutoStuff;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Lift;

public class RetractWithCurrentSensor extends CommandBase {
    private Lift lift;

    public static final int EXTEND_ENCODER_LIMIT_PARTIAL = -5000;
    private static final double RETRACT_POWERLIMIT = 0.9;

    private PIDController liftPID = new PIDController(0.1, 0, 0);
    
    public RetractWithCurrentSensor(Lift lift) {
        this.lift = lift;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        double output = this.lift.LiftCurrentSensor.getAverageVoltage();

        if (Math.abs(output) > 520){
            this.lift.setMotorPower(RETRACT_POWERLIMIT);
        }
    }

}

