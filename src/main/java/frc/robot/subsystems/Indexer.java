package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {
    private static Indexer instance;
    private CANSparkMax master;

    private Indexer() {
        master = new CANSparkMax(RobotMap.Indexer.MASTER_ID, MotorType.kBrushless);

        master.setInverted(RobotMap.Indexer.MASTER_INVERT);

        configMotors();
    }

    public void configMotors() {
        master.restoreFactoryDefaults();

        master.setSmartCurrentLimit(RobotMap.Indexer.CURRENT_LIMIT);
        
        master.burnFlash();
    }

    public void setPower(double power) {
        master.setVoltage(power * RobotMap.MAX_VOLTAGE);
    }
    
    public static Indexer getInstance() {
        if (instance == null) instance = new Indexer();
        return instance;
    }

    
}
