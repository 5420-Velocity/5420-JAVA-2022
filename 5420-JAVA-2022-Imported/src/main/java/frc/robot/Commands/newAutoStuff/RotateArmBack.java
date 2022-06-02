package frc.robot.Commands.newAutoStuff;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.LiftRotationMechanism;

public class RotateArmBack extends CommandBase{
    private LiftRotationMechanism liftRotationMechanism;
    private double liftAngleInput = 0.5;

    public RotateArmBack(LiftRotationMechanism liftRotationMechanism) {
        this.liftRotationMechanism = liftRotationMechanism;
        addRequirements(liftRotationMechanism);
    }

    public void execute(){
        if( liftRotationMechanism.GetLower()){
            this.liftRotationMechanism.setRotationPower(liftAngleInput);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.liftRotationMechanism.setRotationPower(0);
        System.out.println("Finsihed " + this.getName());
    }

    public boolean isFinished() {
        return !liftRotationMechanism.GetLower();
      }
}
