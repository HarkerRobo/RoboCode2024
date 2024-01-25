package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class MoveToPosition extends Command {

    public MoveToPosition() {
        addRequirements(Elevator.getInstance());
    }

    public void execute() {
        switch (Elevator.getInstance().getState()) {
            case IDLE:
                Elevator.getInstance().moveToPosition(0);
                break;
            case TRAP:
                Elevator.getInstance().moveToPosition(RobotMap.Elevator.TRAP_HEIGHT);
                break;
            case STAGE:
                Elevator.getInstance().moveToPosition(RobotMap.Elevator.STAGE_HEIGHT);
                break;
        }
        
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        Elevator.getInstance().moveToPosition(0);
        Elevator.getInstance().setElevatorPower(0);
    }
    
}