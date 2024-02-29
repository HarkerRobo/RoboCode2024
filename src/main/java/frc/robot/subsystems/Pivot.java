package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;

public class Pivot extends SubsystemBase {
    private static Pivot instance; 
    
    private TalonFX master; 

    private DigitalInput limitSwitch;

    private InterpolatingDoubleTreeMap speakerAngles;
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> _appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Angle> _angle = mutable(Degrees.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Angle>> _velocity = mutable(DegreesPerSecond.of(0));

    private Pivot() {
        master = new TalonFX(RobotMap.Pivot.MASTER_ID, RobotMap.CAN_CHAIN);
        
        limitSwitch = new DigitalInput(RobotMap.Pivot.LIMIT_SWITCH_ID);

        speakerAngles = new InterpolatingDoubleTreeMap();
        speakerAngles.put(0.0, 0.0); // TODO

        configMotors();
    }
    
    private void configMotors() {
        master.clearStickyFaults();

        TalonFXConfiguration masterConfig = new TalonFXConfiguration();

        masterConfig.MotorOutput.Inverted = RobotMap.Pivot.MASTER_INVERT;

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        masterConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        masterConfig.TorqueCurrent.PeakReverseTorqueCurrent = 40;

        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = RobotMap.Pivot.PIVOT_FORWARD_SOFT_LIMIT;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RobotMap.Pivot.PIVOT_REVERSE_SOFT_LIMIT;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        
        masterConfig.Feedback.SensorToMechanismRatio = RobotMap.Pivot.PIVOT_ROT_TO_ANGLE;

        masterConfig.Slot0.kP = RobotMap.Pivot.PIVOT_kP;
        masterConfig.Slot0.kS = RobotMap.Pivot.PIVOT_kS;
        masterConfig.Slot0.kV = RobotMap.Pivot.PIVOT_kV;
        masterConfig.Slot0.kA = RobotMap.Pivot.PIVOT_kA;
        masterConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        masterConfig.Slot0.kG = RobotMap.Pivot.PIVOT_kG;

        masterConfig.MotionMagic.MotionMagicCruiseVelocity = RobotMap.Pivot.MAX_CRUISE_VElOCITY;
        masterConfig.MotionMagic.MotionMagicAcceleration = RobotMap.Pivot.MAX_CRUISE_ACCLERATION;

        master.getConfigurator().apply(masterConfig);
    }

    /*
     * Get pivot angle in degrees
     */
    public double getPosition() {
        return master.getPosition().getValue();
    }

    /*
     * Get pivot angle in degrees per second
     */
    public double getVelocity() {
        return master.getVelocity().getValue();
    }

    public double getVoltage() {
        return master.getMotorVoltage().getValueAsDouble();
    }

    public void moveToPosition(double desiredAngle) {
        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(desiredAngle);
        master.setControl(motionMagicVoltage); 
    }
    
    public void setPercentOutput(double power) {
        master.set(power);
    }

    public void setSensorPosition(double degrees) {
        master.getConfigurator().setPosition(degrees);
    }

    public boolean isLimitHit() {
        return !limitSwitch.get();
    }

    public double getPivotSetpoint(double distance) {
        return speakerAngles.get(distance);
    }

    private final SysIdRoutine _sysId = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> {
                        master.setVoltage(volts.in(Volts));
                    },
                    log -> {
                        log.motor("pivot-master")
                                .voltage(
                                        _appliedVoltage.mut_replace(
                                               getVoltage(), Volts))
                                .angularPosition(
                                        _angle.mut_replace(getPosition(), Degrees))
                                .angularVelocity(
                                        _velocity.mut_replace(getVelocity(), DegreesPerSecond));
                    },
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return _sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return _sysId.dynamic(direction);
    }

    public static Pivot getInstance() {
        if(instance == null) {
            instance = new Pivot();
        }
        return instance; 
    }
}
