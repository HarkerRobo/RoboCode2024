package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

        configMotors();
    }
    
    private void configMotors() {
        master.clearStickyFaults();
        follower.clearStickyFaults();

        TalonFXConfiguration masterConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        masterConfig.MotorOutput.Inverted = RobotMap.Elevator.MASTER_INVERT;

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        masterConfig.Voltage.PeakForwardVoltage = RobotMap.MAX_VOLTAGE;
        masterConfig.Voltage.PeakReverseVoltage = -RobotMap.MAX_VOLTAGE;

        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = RobotMap.Elevator.ELEVATOR_FORWARD_SOFT_LIMIT;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RobotMap.Elevator.ELEVATOR_REVERSE_SOFT_LIMIT;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        masterConfig.Slot0.kP = RobotMap.Elevator.ELEVATOR_kP;
        masterConfig.Slot0.kG = RobotMap.Elevator.ELEVATOR_kG;

        follower.setControl(new Follower(RobotMap.Elevator.MASTER_ID, false));

        master.getConfigurator().apply(masterConfig);
        follower.getConfigurator().apply(followerConfig);

        

    }

    public double getPosition() {
        return master.getPosition().getValue();
    }

    public double getVelocity() {
        return master.getVelocity().getValue();
    }

    public void moveToPosition(double desired) {
        PositionVoltage motionMagicVoltage = new PositionVoltage(desired);
        master.setControl(motionMagicVoltage); //IN ROTATIONS 
    }
    
    public void setElevatorPower(double power) {
        DutyCycleOut percentOutput = new DutyCycleOut(power);
        master.setControl(percentOutput);
    }

    public void setSensorPosition(double position) {
        master.getConfigurator().setPosition(position);
    }

    public boolean isLimitHit() {
        return !limitSwitch.get();
    }

    public boolean isAtTop() {
        return master.getPosition().getValue() >= RobotMap.Elevator.STAGE_HEIGHT;
    }

    public boolean isStalling() {
        return master.getStatorCurrent().getValue() >= RobotMap.Elevator.ELEVATOR_STALLING_CURRENT;
    }

    public static Elevator getInstance() {
        if(instance == null) {
            instance = new Elevator();
        }
        return instance; 
    }
    
    public TalonFX getMaster() {
        return master; 
    }
    public TalonFX getFollower() {
        return follower;
    }
}