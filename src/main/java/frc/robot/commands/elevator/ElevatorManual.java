package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class ElevatorManual extends Command{
    public ElevatorManual() {
        addRequirements(Elevator.getInstance());
    }

    public void execute() {
        Elevator.getInstance().setElevatorPower(0.2);
    }

    public boolean isFinished() {
        return Elevator.getInstance().isLimitHit();
    }

    public void end(boolean interrupted) {
        Elevator.getInstance().setElevatorPower(0);
        Elevator.getInstance().setSensorPosition(0);
    }
}