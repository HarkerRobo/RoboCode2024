package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private static Shooter instance;
    private TalonFX master;
    private TalonFX follower;

    private CANSparkMax indexer;

    private DigitalInput proxSensor;

    private Shooter() {
        master = new TalonFX(RobotMap.Shooter.MASTER_ID);
        follower = new TalonFX(RobotMap.Shooter.FOLLOWER_ID);
        indexer = new CANSparkMax(RobotMap.Shooter.INDEXER_ID, MotorType.kBrushless);

        proxSensor = new DigitalInput(RobotMap.Shooter.PROX_SENSOR_ID);

        configShooter();
        configIndexer();
    }

    public void configIndexer() {
        indexer.restoreFactoryDefaults();
        indexer.setInverted(RobotMap.Shooter.INDEXER_INVERT);
        indexer.setSmartCurrentLimit(RobotMap.Shooter.INDEXER_CURRENT_LIMIT);
        indexer.setIdleMode(IdleMode.kCoast);
        indexer.burnFlash();
    }

    public void configShooter() {
        master.clearStickyFaults();
        follower.clearStickyFaults();

        TalonFXConfiguration masterConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        masterConfig.Voltage.PeakForwardVoltage = RobotMap.MAX_VOLTAGE;
        masterConfig.Voltage.PeakReverseVoltage = -RobotMap.MAX_VOLTAGE;

        masterConfig.CurrentLimits.SupplyCurrentLimit = RobotMap.Shooter.SHOOTER_CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentThreshold = RobotMap.Shooter.SHOOTER_CURRENT_LIMIT_THRESHOLD;
        masterConfig.CurrentLimits.SupplyTimeThreshold = RobotMap.Shooter.SHOOTER_CURRENT_LIMIT_TIME;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        master.getConfigurator().apply(masterConfig);
        follower.getConfigurator().apply(followerConfig);

        follower.setControl(new Follower(RobotMap.Elevator.MASTER_ID, true));
        // master.restoreFactoryDefaults();
        // follower.restoreFactoryDefaults();

        // master.setSmartCurrentLimit(RobotMap.Shooter.SHOOTER_CURRENT_LIMIT);

        // master.setIdleMode(IdleMode.kCoast);
        // follower.setIdleMode(IdleMode.kCoast);

        // follower.follow(master);

        // master.burnFlash();
        // follower.burnFlash();
    }

    public void setShooter(double power) {
        master.setVoltage(power * RobotMap.MAX_VOLTAGE);
    }

    public void setIndexer(double power) {
        indexer.setVoltage(power * RobotMap.MAX_VOLTAGE);
    }

    public boolean shooterIndexerOccupied() {
        return !proxSensor.get();
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }
    
}
