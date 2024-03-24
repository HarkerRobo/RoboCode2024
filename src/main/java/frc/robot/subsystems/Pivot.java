package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;
import frc.robot.util.MathUtil;
// import frc.robot.util.Telemetry;

public class Pivot extends SubsystemBase {
    private static Pivot instance; 
    
    private TalonFX master; 

    private DigitalInput limitSwitch;

    private CANcoder canCoder;

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

        canCoder = new CANcoder(RobotMap.Pivot.CAN_CODER_ID);

        speakerAngles = new InterpolatingDoubleTreeMap();
        speakerAngles.put(0.0, 22.5);
        speakerAngles.put(1.787, 22.5);
        speakerAngles.put(2.043, 35.571);
        speakerAngles.put(2.361, 37.5);
        speakerAngles.put(2.839, 39.643);
        speakerAngles.put(3.228, 42.574);
        speakerAngles.put(3.713, 45.914);
        speakerAngles.put(4.156, 46.5);
        speakerAngles.put(4.507, 47.463);
        speakerAngles.put(5.051, 48.990);
        configCANcoder();
        configMotors();
    }

    private void configMotors() {
        master.clearStickyFaults();

        TalonFXConfiguration masterConfig = new TalonFXConfiguration();

        masterConfig.MotorOutput.Inverted = RobotMap.Pivot.MASTER_INVERT;

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // masterConfig.Feedback.SensorToMechanismRatio = RobotMap.Pivot.PIVOT_GEAR_RATIO;

        masterConfig.Slot0.kP = RobotMap.Pivot.PIVOT_kP;
        masterConfig.Slot0.kI = RobotMap.Pivot.PIVOT_kI;
        masterConfig.Slot0.kD = RobotMap.Pivot.PIVOT_kD;
        masterConfig.Slot0.kS = RobotMap.Pivot.PIVOT_kS;

        masterConfig.Slot1.kP = RobotMap.Pivot.PIVOT_AMP_kP;
        masterConfig.Slot1.kS = RobotMap.Pivot.PIVOT_AMP_kS;

        masterConfig.MotionMagic.MotionMagicCruiseVelocity = RobotMap.Pivot.MAX_CRUISE_VElOCITY;
        masterConfig.MotionMagic.MotionMagicAcceleration = RobotMap.Pivot.MAX_CRUISE_ACCLERATION;

        masterConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        masterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        master.getConfigurator().apply(masterConfig);
    }

    private void configCANcoder() {
        canCoder.clearStickyFaults();

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset = -RobotMap.Pivot.CAN_CODER_OFFSET; // offset is ADDED, so -offset

        canCoder.getConfigurator().apply(canCoderConfig);
    }

    /*
     * Get pivot angle in degrees
     */
    public double getPosition() {
        return canCoder.getAbsolutePosition().getValue() * RobotMap.Pivot.PIVOT_ROT_TO_ANGLE;
    }

    public boolean isStalling() {
        return master.getStatorCurrent().getValue() > RobotMap.Pivot.STALLING_CURRENT;
    }

    public double getMasterCurrent() {
        return master.getStatorCurrent().getValue();
    }

    /*
     * Get pivot angle in degrees per second
     */
    public double getVelocity() {
        return canCoder.getVelocity().getValue() * RobotMap.Pivot.PIVOT_ROT_TO_ANGLE;
    }

    public double getVoltage() {
        return master.getMotorVoltage().getValueAsDouble();
    }

    public void moveToPosition(double desiredAngle) {
        double kG = RobotMap.Pivot.PIVOT_kG * Math.cos(Math.toRadians(getPosition() + RobotMap.Pivot.OFFSET_ANGLE));
        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(desiredAngle / RobotMap.Pivot.PIVOT_ROT_TO_ANGLE);
        master.setControl(motionMagicVoltage.withFeedForward(kG)); 
    }

    public void moveToPositionAmp(double desiredAngle) {
        double kG = RobotMap.Pivot.PIVOT_kG * Math.cos(Math.toRadians(getPosition() + RobotMap.Pivot.OFFSET_ANGLE));
        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(desiredAngle / RobotMap.Pivot.PIVOT_ROT_TO_ANGLE);
        master.setControl(motionMagicVoltage.withFeedForward(kG).withSlot(1)); 
    }
    
    public void setPercentOutput(double power) {
        if (MathUtil.compareDouble(power, 0))
        {
            master.stopMotor();
        }
        DutyCycleOut percentOutput = new DutyCycleOut(power);
        master.setControl(percentOutput);
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
