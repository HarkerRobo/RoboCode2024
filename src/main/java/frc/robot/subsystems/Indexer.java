package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {
    private static Indexer instance;
    private TalonFX master;

    private Indexer() {
        master = new TalonFX(RobotMap.Indexer.MASTER_ID);

        configMotors();
    }

    public void configMotors() {
        master.clearStickyFaults();

        TalonFXConfiguration masterConfiguration = new TalonFXConfiguration();
        masterConfiguration.MotorOutput.Inverted = RobotMap.Indexer.MASTER_INVERT;

        masterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        masterConfiguration.CurrentLimits.SupplyCurrentLimit = RobotMap.Indexer.CURRENT_LIMIT;
        masterConfiguration.CurrentLimits.SupplyCurrentThreshold = RobotMap.Indexer.CURRENT_LIMIT_THRESHOLD;
        masterConfiguration.CurrentLimits.SupplyTimeThreshold = RobotMap.Indexer.CURRENT_LIMIT_TIME;
        masterConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        master.getConfigurator().apply(masterConfiguration);
    }

    public void setPower(double power) {
        master.setVoltage(power * RobotMap.MAX_VOLTAGE);
    }
    
    public static Indexer getInstance() {
        if (instance == null) instance = new Indexer();
        return instance;
    }

    
}
