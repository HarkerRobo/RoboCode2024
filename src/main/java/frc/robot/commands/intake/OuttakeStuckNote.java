package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class OuttakeStuckNote extends Command {

    public OuttakeStuckNote()
    {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void execute() {
        Intake.getInstance().setDeployPos(RobotMap.Intake.INTAKE_DEPLOY);
        Intake.getInstance().setRollerPower(RobotMap.Intake.ROLLER_OUTAKE_SPEED);
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().setDeployPos(0);
        Intake.getInstance().setRollerPower(0);
    }
    
}
