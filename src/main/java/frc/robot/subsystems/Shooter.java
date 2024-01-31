package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private static Shooter instance;
    private CANSparkMax master;
    private CANSparkMax follower;

    private CANSparkMax indexer;

    private DigitalInput proxSensor;

    private Shooter() {
        master = new CANSparkMax(RobotMap.Shooter.MASTER_ID, MotorType.kBrushless);
        follower = new CANSparkMax(RobotMap.Shooter.FOLLOWER_ID, MotorType.kBrushless);
        indexer = new CANSparkMax(RobotMap.Shooter.INDEXER_ID, MotorType.kBrushless);

        proxSensor = new DigitalInput(RobotMap.Shooter.PROX_SENSOR_ID);

        master.setInverted(RobotMap.Shooter.MASTER_INVERT);
        follower.setInverted(RobotMap.Shooter.FOLLOWER_INVERT);
        indexer.setInverted(RobotMap.Shooter.INDEXER_INVERT);

        configMotor();
    }

    public void configMotor() {
        master.restoreFactoryDefaults();
        follower.restoreFactoryDefaults();

        master.setSmartCurrentLimit(RobotMap.Shooter.SHOOTER_CURRENT_LIMIT);
        indexer.setSmartCurrentLimit(RobotMap.Shooter.INDEXER_CURRENT_LIMIT);

        master.setIdleMode(IdleMode.kCoast);
        follower.setIdleMode(IdleMode.kCoast);
        indexer.setIdleMode(IdleMode.kBrake);

        follower.follow(master);

        master.burnFlash();
        follower.burnFlash();
        indexer.burnFlash();
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
