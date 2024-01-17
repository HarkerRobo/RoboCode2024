package frc.robot;

import javax.xml.crypto.dsig.XMLObject;

import frc.robot.util.XboxGamepad;

public class OI {
    private static OI instance; 

    private XboxGamepad driver;
    private XboxGamepad operator;
    public OI() {
        driver = new XboxGamepad(RobotMap.OI.DRIVER_ID);
        operator = new XboxGamepad(RobotMap.OI.OPERATOR_ID);
    }   
    public XboxGamepad getDriver() {
        return driver;
    }
    public XboxGamepad getOperator() {
        return operator;
    }
    private void initBindings() {

    }
    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }
}