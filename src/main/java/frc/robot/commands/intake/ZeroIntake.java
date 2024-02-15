package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class ZeroIntake extends Command {
    public ZeroIntake() {
        addRequirements(Intake.getInstance());
    }

    public void execute() {
        Intake.getInstance().setDeployPower(RobotMap.Intake.ZERO_SPEED);
    }

    public boolean isFinished() {
        return Intake.getInstance().limitSwitchHit();
    }

    public void end(boolean interrupted) {
        Intake.getInstance().setDeployPower(0);
        Intake.getInstance().setSensorPosition(0);
    }
    
}