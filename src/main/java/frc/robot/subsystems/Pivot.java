package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
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

    private RobotMap.Pivot.Goal goal;

    private Pivot() {
        master = new TalonFX(RobotMap.Pivot.MASTER_ID);
        follower = new TalonFX(RobotMap.Pivot.FOLLOWER_ID); 

        master.setInverted(RobotMap.Pivot.MASTER_INVERT);
        follower.setInverted(RobotMap.Pivot.FOLLOWER_INVERT);

        speakerAngles = new InterpolatingDoubleTreeMap();
        speakerAngles.put(0.0, 0.0);

        goal = RobotMap.Pivot.Goal.INTAKE;

        configMotors();
    }
    
    private void configMotors() {
        master.clearStickyFaults();
        follower.clearStickyFaults();

        TalonFXConfiguration config = new TalonFXConfiguration();

        SoftwareLimitSwitchConfigs softlimits = new SoftwareLimitSwitchConfigs();
        softlimits.ForwardSoftLimitThreshold = RobotMap.Pivot.PIVOT_FORWARD_SOFT_LIMIT;
        softlimits.ReverseSoftLimitThreshold = RobotMap.Pivot.PIVOT_REVERSE_SOFT_LIMIT;
        softlimits.ForwardSoftLimitEnable = true;
        softlimits.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch = softlimits;

        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput = motorConfigs;

        Slot0Configs pivotPID = new Slot0Configs();
        pivotPID.kP = RobotMap.Pivot.PIVOT_kP;
        pivotPID.kS = RobotMap.Pivot.PIVOT_kS;
        pivotPID.kV = RobotMap.Pivot.PIVOT_kV;
        pivotPID.kA = RobotMap.Pivot.PIVOT_kA;
        pivotPID.GravityType = GravityTypeValue.Arm_Cosine;
        pivotPID.kG = RobotMap.Pivot.PIVOT_kG;
        config.Slot0 = pivotPID;

        MotionMagicConfigs magicConfigs = new MotionMagicConfigs();
        magicConfigs.MotionMagicCruiseVelocity = RobotMap.Pivot.MAX_CRUISE_VElOCITY;
        magicConfigs.MotionMagicAcceleration = RobotMap.Pivot.MAX_CRUISE_ACCLERATION;
        config.MotionMagic = magicConfigs;

        master.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);

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

    public void setGoal(RobotMap.Pivot.Goal goal) {
        this.goal = goal;
    }

    public RobotMap.Pivot.Goal getGoal() {
        return goal;
    }

    public static Pivot getInstance() {
        if(instance == null) {
            instance = new Pivot();
        }
        return instance; 
    }
}
