package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class SwerveModule {
    //motors on the swerve modules
    private TalonFX translation;
    private TalonFX rotation;

    private CANcoder canCoder;

    //swerve module id
    private int ID;

    private static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(RobotMap.SwerveModule.TRANSLATION_kS, RobotMap.SwerveModule.TRANSLATION_kV, RobotMap.SwerveModule.TRANSLATION_kA);

    public SwerveModule(int id) {
        ID = id;

        //configures translation and rotation motors
        translation = new TalonFX(RobotMap.SwerveModule.TRANSLATION_IDS[id], RobotMap.CAN_CHAIN);

        rotation = new TalonFX(RobotMap.SwerveModule.ROTATION_IDS[id], RobotMap.CAN_CHAIN);
        
        canCoder = new CANcoder(RobotMap.SwerveModule.CAN_CODER_ID[id], RobotMap.CAN_CHAIN);
        
        configCANcoder();
        configTranslation();
        configRotation();
        setAbsolutePosition();
    }   

    private void configTranslation() {
        translation.clearStickyFaults();

        TalonFXConfiguration transConfig = new TalonFXConfiguration();

        CurrentLimitsConfigs transCurrentConfig = new CurrentLimitsConfigs();
        transCurrentConfig.SupplyCurrentLimit = RobotMap.SwerveModule.TRANS_CURRENT_LIMIT;
        transCurrentConfig.SupplyCurrentThreshold = RobotMap.SwerveModule.TRANS_THRESHOLD_CURRENT;
        transCurrentConfig.SupplyTimeThreshold = RobotMap.SwerveModule.TRANS_THRESHOLD_TIME;
        transCurrentConfig.SupplyCurrentLimitEnable = true;
        transConfig.CurrentLimits = transCurrentConfig;

        MotorOutputConfigs transOutputConfig = new MotorOutputConfigs();
        transOutputConfig.NeutralMode = NeutralModeValue.Brake;
        transConfig.MotorOutput = transOutputConfig;

        Slot0Configs transPID = new Slot0Configs();
        transPID.kP = RobotMap.SwerveModule.TRANSLATION_kP;
        transPID.kI = RobotMap.SwerveModule.TRANSLATION_kI;
        transPID.kD = RobotMap.SwerveModule.TRANSLATION_kD;
        transConfig.Slot0 = transPID;

        translation.getConfigurator().apply(transConfig);

        translation.setInverted(RobotMap.SwerveModule.TRANSLATION_INVERTS[ID]);
    }

    private void configRotation() {
        rotation.clearStickyFaults();

        TalonFXConfiguration rotConfig = new TalonFXConfiguration();

        CurrentLimitsConfigs rotCurrentConfig = new CurrentLimitsConfigs();
        rotCurrentConfig.SupplyCurrentLimit = RobotMap.SwerveModule.ROT_CURRENT_LIMIT;
        rotCurrentConfig.SupplyCurrentThreshold = RobotMap.SwerveModule.ROT_THRESHOLD_CURRENT;
        rotCurrentConfig.SupplyTimeThreshold = RobotMap.SwerveModule.ROT_THRESHOLD_TIME;
        rotConfig.CurrentLimits = rotCurrentConfig;

        MotorOutputConfigs rotOutputConfig = new MotorOutputConfigs();
        rotOutputConfig.NeutralMode = NeutralModeValue.Brake;
        rotConfig.MotorOutput = rotOutputConfig;

        Slot0Configs rotPID = new Slot0Configs();
        rotPID.kP = RobotMap.SwerveModule.ROTATION_kP;
        rotConfig.Slot0 = rotPID;

        rotation.getConfigurator().apply(rotConfig);

        rotation.setInverted(RobotMap.SwerveModule.ROTATION_INVERTS[ID]);
    }

    private void configCANcoder() {
        canCoder.clearStickyFaults();

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset = RobotMap.SwerveModule.CAN_CODER_OFFSETS[ID];
        
        canCoder.getConfigurator().apply(canCoderConfig);
    }
    /**
     * Sets translation and rotation motors to move to new state
     * @param state new state 
     */
    public void setAngleAndDrive(SwerveModuleState state) {
        state = optimize(state);

        VelocityVoltage transRequest = new VelocityVoltage(state.speedMetersPerSecond / RobotMap.SwerveModule.TRANS_ROT_TO_METERS);
        transRequest.FeedForward = feedforward.calculate(state.speedMetersPerSecond);
        translation.setControl(transRequest);

        PositionDutyCycle rotRequest = new PositionDutyCycle(state.angle.getDegrees() / RobotMap.SwerveModule.ROT_ROT_TO_ANGLE);
        rotation.setControl(rotRequest);
    }
    /*
     * adjusts the angle of a swerve module state 
     */
    public SwerveModuleState optimize (SwerveModuleState desiredState){
       double currentAngle = Math.toRadians(getAngle());
       double targetAngle = Math.IEEEremainder(desiredState.angle.getRadians(), Math.PI * 2);
       double remainder = currentAngle % (Math.PI * 2);
        var adjusted = targetAngle + currentAngle - remainder;

        var speed = desiredState.speedMetersPerSecond;
        SmartDashboard.putNumber(swerveIDToName(ID) + "Desired Translation Speed", speed);
        
        if(adjusted - currentAngle > Math.PI) {
            adjusted -= Math.PI * 2;
        }
        if (adjusted - currentAngle < -Math.PI) {
            adjusted += Math.PI * 2;
        }
        if (adjusted - currentAngle > Math.PI / 2) {
            adjusted -= Math.PI;
            speed *= -1;
        } else if (adjusted - currentAngle < -Math.PI / 2) {
            adjusted += Math.PI;
            speed *= -1;
        }
        return new SwerveModuleState(speed, Rotation2d.fromRadians(adjusted));
        
    }
    /*
     * resets the swerve module
     */
    private void setAbsolutePosition() {
        double position = canCoder.getAbsolutePosition().getValue(); // rotations
        rotation.getConfigurator().setPosition(position); // rotations
        zeroTranslation();
    }

    public void zeroTranslation() {
        translation.getConfigurator().setPosition(0); // rotations
    }
    
    /*
     * returns the angle of the rotation motor 
     */
    public double getAngle() {
        return rotation.getRotorPosition().getValue() * RobotMap.SwerveModule.ROT_ROT_TO_ANGLE;
    }

    /*
     * return speed of translation motor 
     */
    public double getSpeed() {
        return translation.getRotorVelocity().getValue() * RobotMap.SwerveModule.TRANS_ROT_TO_METERS;
    }

    /*
     * returns position of translation motor
     */
    public double getWheelPosition() {
        return translation.getRotorPosition().getValue() * RobotMap.SwerveModule.TRANS_ROT_TO_METERS;
    }

    //returns position and angle
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getWheelPosition(), Rotation2d.fromDegrees(getAngle()));
    }

    //returns speed and angle
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getAngle()));
    }
    
    //name of module on smart dashbaord 
    public static String swerveIDToName(int swerveID) {
        String output = "";
        if (swerveID < 2) output += "Front ";
        else output += "Back ";
        if (swerveID % 2 == 0) output += "Left";
        else output += "Right";
        return output;
    }
}