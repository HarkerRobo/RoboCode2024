package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {

    public IntakeNote() {
        addRequirements(Intake.getInstance());
    }

    public void execute() {
        // Intake.getInstance().setDeployPos(RobotMap.Intake.INTAKE_DEPLOY);
        Intake.getInstance().setRollerPower(RobotMap.Intake.ROLLER_SPEED);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        Intake.getInstance().setRollerPower(0);
        // Intake.getInstance().setDeployPos(0);
    }
    
}
