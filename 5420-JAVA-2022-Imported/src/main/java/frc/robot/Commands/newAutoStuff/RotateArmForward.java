package frc.robot.Commands.newAutoStuff;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Lift;

public class RotateArmForward extends CommandBase{
    private Lift lift;
    private double liftAngleInput = -0.25;

    public RotateArmForward(Lift lift) {
        this.lift = lift;
        addRequirements(lift);
    }

    public void execute(){
        if(lift.GetUpper()) {
            this.lift.setRotationPower(liftAngleInput);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.lift.setRotationPower(0);
        System.out.println("Finsihed " + this.getName());
    }

    public boolean isFinished() {
        return !lift.GetUpper();
    }
}


