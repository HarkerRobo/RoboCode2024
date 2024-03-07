package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class ElevatorDown extends Command{
    public ElevatorDown() {
        addRequirements(Elevator.getInstance());
    }

    @Override
    public void execute() {
        Elevator.getInstance().setElevatorPower(-0.3);
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Elevator.getInstance().setElevatorPower(0);
    }
}
