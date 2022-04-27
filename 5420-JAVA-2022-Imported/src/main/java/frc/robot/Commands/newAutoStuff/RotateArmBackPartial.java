package frc.robot.Commands.newAutoStuff;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Lift;

public class RotateArmBackPartial extends CommandBase{
    private Lift lift;
    private double liftAngleInput = 0.5;

    public RotateArmBackPartial(Lift lift) {
        this.lift = lift;

    }

    public void execute(){
        // Set the motor speed if the Upper Limit (invered) is detected
        if(!lift.GetUpper()) {
            this.lift.setRotationPower(liftAngleInput);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.lift.setRotationPower(0);
        System.out.println("Finsihed " + this.getName());
    }


    public boolean isFinished() {
        return !lift.GetLower();
    }
}
