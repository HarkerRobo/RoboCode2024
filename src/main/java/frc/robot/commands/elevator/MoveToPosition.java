package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;
import frc.robot.util.MathUtil;

public class MoveToPosition extends Command {

    private double setpoint;

    public MoveToPosition(double desired) {
        this.setpoint = desired;
        addRequirements(Elevator.getInstance());
    }

    public void execute() {
        Elevator.getInstance().moveToPosition(setpoint);
        
    }

    public boolean isFinished() {
        return MathUtil.compareSetpoint(Elevator.getInstance().getPosition(), setpoint, RobotMap.Elevator.MAX_ERROR);
    }

    // public void end(boolean interrupted) {
    //     Elevator.getInstance().moveToPosition(0);
    //     Elevator.getInstance().setElevatorPower(0);
    // }
    
}