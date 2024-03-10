package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class ElevatorManual extends Command{
    private double power;

    public ElevatorManual(double power) {
        this.power = power;
        addRequirements(Elevator.getInstance());
    }

    @Override
    public void execute() {
        Elevator.getInstance().setElevatorPower(power);
    }

    public boolean isFinished() {
        if (power > 0)
            return Elevator.getInstance().isAtTop();
        else
            return Elevator.getInstance().isStalling();
    }

    @Override
    public void end(boolean interrupted) {
        if (power > 0)
            Elevator.getInstance().setElevatorPower(RobotMap.Elevator.ELEVATOR_kG);
        else
            Elevator.getInstance().setElevatorPower(0);
    }
}
