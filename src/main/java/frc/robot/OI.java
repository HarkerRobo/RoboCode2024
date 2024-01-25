package frc.robot;

import frc.robot.commands.CommandGroups;
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
        operator.getLeftBumper().onTrue(CommandGroups.FULL_SHOOT_SPEAKER);
        operator.getRightBumper().onTrue(CommandGroups.FULL_INTAKE);
        operator.getButtonY().onTrue(CommandGroups.FULL_SHOOT_AMP);

        operator.getRightDPadButton().onTrue(CommandGroups.PRE_ALIGN_CLIMB);
        operator.getUpDPadButton().onTrue(CommandGroups.POST_ALIGN_CLIMB);
        operator.getDownDPadButton().onTrue(CommandGroups.FULL_SHOOT_TRAP);
    }

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }
}