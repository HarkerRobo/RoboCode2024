package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.CommandGroups;
import frc.robot.commands.drivetrain.AlignToStage;
import frc.robot.util.XboxGamepad;

public class OI {
    private static OI instance; 

    private XboxGamepad driver;
    private XboxGamepad operator;
    public OI() {
        driver = new XboxGamepad(RobotMap.OI.DRIVER_ID);
        operator = new XboxGamepad(RobotMap.OI.OPERATOR_ID);
        initBindings();
    }  

    public XboxGamepad getDriver() {
        return driver;
    }

    public XboxGamepad getOperator() {
        return operator;
    }

    private void initBindings() {
        driver.getLeftBumper().onTrue(CommandGroups.FULL_SHOOT_AMP);
        driver.getRightBumper().onTrue(CommandGroups.FULL_SHOOT_SPEAKER);
        if (driver.getLeftTrigger() > 0.5) CommandScheduler.getInstance().schedule(new AlignToStage());

        operator.getRightBumper().onTrue(CommandGroups.FULL_INTAKE);
        operator.getUpDPadButton().onTrue(CommandGroups.PRE_ALIGN_CLIMB);
        operator.getDownDPadButton().onTrue(CommandGroups.POST_ALIGN_CLIMB);
        operator.getRightDPadButton().onTrue(CommandGroups.FULL_SHOOT_TRAP);
    }

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }
}