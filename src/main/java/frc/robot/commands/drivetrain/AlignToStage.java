package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.Limelight;

public class AlignToStage extends Command {

    private static PIDController vxStageController = new PIDController(RobotMap.Drivetrain.VX_STAGE_kP, 0, 0);
    private static PIDController vyStageController = new PIDController(RobotMap.Drivetrain.VY_STAGE_kP, 0, 0);

    public AlignToStage() {
        addRequirements(Drivetrain.getInstance());
        vxStageController.setTolerance(RobotMap.Drivetrain.MAX_ERROR_DEG_TX_STAGE);
        vyStageController.setTolerance(RobotMap.Drivetrain.MAX_ERROR_DEG_TY_STAGE);
    }

    public void initailize() {
        vxStageController.reset();
        vyStageController.reset();
        vxStageController.setSetpoint(0);
        vyStageController.setSetpoint(RobotMap.Drivetrain.VERTICAL_DEG_STAGE);
    }

    public void execute() {
        double vx = vxStageController.calculate(Limelight.getTx());
        double vy = vyStageController.calculate(Limelight.getTy());

        ChassisSpeeds chassis = new ChassisSpeeds(vx, vy, 0);
        Drivetrain.getInstance().setAngleAndDrive(chassis);
    }


    public boolean isFinished() {
        return vxStageController.atSetpoint() && vyStageController.atSetpoint();
    }

    public void end(boolean interrupted) {
        Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
    }


}
