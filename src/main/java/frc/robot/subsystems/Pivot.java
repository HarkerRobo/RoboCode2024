package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Pivot extends SubsystemBase {
    private static Pivot instance; 
    
    private TalonFX master; 
    private TalonFX follower; 

    private DigitalInput limitSwitch;

    private InterpolatingDoubleTreeMap speakerAngles;

    private Pivot() {
        master = new TalonFX(RobotMap.Pivot.MASTER_ID, RobotMap.CAN_CHAIN);
        follower = new TalonFX(RobotMap.Pivot.FOLLOWER_ID, RobotMap.CAN_CHAIN); 

        speakerAngles = new InterpolatingDoubleTreeMap();
        speakerAngles.put(0.0, 0.0); // TODO

        configMotors();
    }
    
    private void configMotors() {
        master.clearStickyFaults();
        follower.clearStickyFaults();

        TalonFXConfiguration masterConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        masterConfig.MotorOutput.Inverted = RobotMap.Pivot.MASTER_INVERT;
        followerConfig.MotorOutput.Inverted = RobotMap.Pivot.FOLLOWER_INVERT;

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = RobotMap.Pivot.PIVOT_FORWARD_SOFT_LIMIT;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RobotMap.Pivot.PIVOT_REVERSE_SOFT_LIMIT;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        masterConfig.Slot0.kP = RobotMap.Pivot.PIVOT_kP;
        masterConfig.Slot0.kS = RobotMap.Pivot.PIVOT_kS;
        masterConfig.Slot0.kV = RobotMap.Pivot.PIVOT_kV;
        masterConfig.Slot0.kA = RobotMap.Pivot.PIVOT_kA;
        masterConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        masterConfig.Slot0.kG = RobotMap.Pivot.PIVOT_kG;

        masterConfig.MotionMagic.MotionMagicCruiseVelocity = RobotMap.Pivot.MAX_CRUISE_VElOCITY;
        masterConfig.MotionMagic.MotionMagicAcceleration = RobotMap.Pivot.MAX_CRUISE_ACCLERATION;

        master.getConfigurator().apply(masterConfig);
        follower.getConfigurator().apply(followerConfig);

        follower.setControl(new Follower(RobotMap.Pivot.MASTER_ID, false));
    }

    public double getPosition() {
        return master.getPosition().getValue() * RobotMap.Pivot.PIVOT_ROT_TO_ANGLE;
    }

    public void moveToPosition(double desiredAngle) {
        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(desiredAngle / RobotMap.Pivot.PIVOT_ROT_TO_ANGLE);
        master.setControl(motionMagicVoltage); 
    }
    
    public void setPercentOutput(double power) {
        master.set(power);
    }

    public void setSensorPosition(double degrees) {
        master.getConfigurator().setPosition(degrees / RobotMap.Pivot.PIVOT_ROT_TO_ANGLE);
    }

    public boolean isLimitHit() {
        return !limitSwitch.get();
    }

    public double getPivotSetpoint(double distance) {
        return speakerAngles.get(distance);
    }

    public static Pivot getInstance() {
        if(instance == null) {
            instance = new Pivot();
        }
        return instance; 
    }
}
