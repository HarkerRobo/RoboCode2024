package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
    private static Elevator instance; 
    
    private TalonFX master; 
    private TalonFX follower; 

    private DigitalInput limitSwitch; 

    private Elevator() {
        master = new TalonFX(RobotMap.Elevator.MASTER_ID, RobotMap.CAN_CHAIN);
        follower = new TalonFX(RobotMap.Elevator.FOLLOWER_ID, RobotMap.CAN_CHAIN); 

        limitSwitch = new DigitalInput(RobotMap.Elevator.LIMIT_SWITCH_ID);

        master.setInverted(RobotMap.Elevator.MASTER_INVERT);
        follower.setInverted(RobotMap.Elevator.FOLLOWER_INVERT);

        configMotors();
    }
    
    private void configMotors() {
        master.clearStickyFaults();
        follower.clearStickyFaults();

        TalonFXConfiguration config = new TalonFXConfiguration();

        SoftwareLimitSwitchConfigs softlimits = new SoftwareLimitSwitchConfigs();
        softlimits.ForwardSoftLimitThreshold = RobotMap.Elevator.ELEVATOR_FORWARD_SOFT_LIMIT;
        softlimits.ReverseSoftLimitThreshold = RobotMap.Elevator.ELEVATOR_REVERSE_SOFT_LIMIT;
        softlimits.ForwardSoftLimitEnable = true;
        softlimits.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch = softlimits;

        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput = motorConfigs;

        Slot0Configs elevatorPID = new Slot0Configs();
        elevatorPID.kP = RobotMap.Elevator.ELEVATOR_kP;
        elevatorPID.kG = RobotMap.Elevator.ELEVATOR_kG;
        config.Slot0 = elevatorPID;

        master.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);

        follower.setControl(new Follower(RobotMap.Elevator.MASTER_ID, false));
    }

    public double getPosition() {
        return master.getPosition().getValue();
    }

    public void moveToPosition(double desired) {
        PositionVoltage motionMagicVoltage = new PositionVoltage(desired);
        master.setControl(motionMagicVoltage); //IN ROTATIONS 
    }
    
    public void setElevatorPower(double power) {
        master.set(power);
    }

    public void setSensorPosition(double position) {
        master.getConfigurator().setPosition(position);
    }

    public boolean isLimitHit() {
        return !limitSwitch.get();
    }

    public static Elevator getInstance() {
        if(instance == null) {
            instance = new Elevator();
        }
        return instance; 
    }
    
}