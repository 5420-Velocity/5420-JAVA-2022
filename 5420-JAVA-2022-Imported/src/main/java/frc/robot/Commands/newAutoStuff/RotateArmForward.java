package frc.robot.Commands.newAutoStuff;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Commands.liftRotationControl;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.LiftRotationMechanism;

public class RotateArmForward extends CommandBase{
    private Lift lift;
    private double liftAngleInput = -0.25;
    private LiftRotationMechanism LiftRotationMechanism;

    public RotateArmForward(LiftRotationMechanism liftRotationMechanism) {
        this.LiftRotationMechanism = liftRotationMechanism;
        addRequirements(liftRotationMechanism);
    }

    public void execute(){
        if(LiftRotationMechanism.GetUpper()) {
            this.LiftRotationMechanism.setRotationPower(liftAngleInput);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.LiftRotationMechanism.setRotationPower(0);
        System.out.println("Finsihed " + this.getName());
    }

    public boolean isFinished() {
        return !LiftRotationMechanism.GetUpper();
    }
}


