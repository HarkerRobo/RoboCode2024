package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class OuttakeNote extends Command {

    public OuttakeNote()
    {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void execute() {
        Intake.getInstance().setDeployPos(0);
        Intake.getInstance().setRollerPower(-RobotMap.Intake.ROLLER_SPEED);
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().setRollerPower(0);
    }
    
}
