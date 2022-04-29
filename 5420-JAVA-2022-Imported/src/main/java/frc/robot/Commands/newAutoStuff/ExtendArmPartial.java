package frc.robot.Commands.newAutoStuff;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Lift;

public class ExtendArmPartial extends CommandBase {
    private Lift lift;

    public static final int EXTEND_ENCODER_LIMIT_PARTIAL = -5000;
    private static final double EXTEND_POWERLIMIT = -0.5;

    private PIDController liftPID = new PIDController(0.1, 0, 0);
    
    public ExtendArmPartial(Lift lift) {
        this.lift = lift;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        double output = Math.abs(liftPID.calculate(lift.GetLiftEncoder(), EXTEND_ENCODER_LIMIT_PARTIAL));
        if (Math.abs(output) > 1) {
            output = 1 * Math.signum(output);
        }
        this.lift.setMotorPower(EXTEND_POWERLIMIT);
    }

    @Override
    public void end(boolean interrupted) {
        // When the comammand has been ended OR was canceled, stop the motor
        this.lift.setMotorPower(0);
        System.out.println("Finsihed " + this.getName());
    }

    @Override
    public boolean isFinished() {
        return this.lift.GetLiftEncoder() < EXTEND_ENCODER_LIMIT_PARTIAL;
    }
}
