package frc.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    private SimpleMotorFeedforward feedforward;

    public SwerveModule(int id) {
        ID = id;

        //configures translation and rotation motors
        translation = new TalonFX(RobotMap.SwerveModule.TRANSLATION_IDS[id], RobotMap.CAN_CHAIN);

        rotation = new TalonFX(RobotMap.SwerveModule.ROTATION_IDS[id],RobotMap.CAN_CHAIN);
        
        canCoder = new CANcoder(RobotMap.SwerveModule.CAN_CODER_ID[id]);

        feedforward = new SimpleMotorFeedforward(RobotMap.SwerveModule.TRANSLATION_KS, RobotMap.SwerveModule.TRANSLATION_KV, RobotMap.SwerveModule.TRANSLATION_KA);
        
        configTranslation();
        configRotation();
        configCANcoder();
        setAbsolutePosition();
    }   

    private void configTranslation() {
        translation.clearStickyFaults();

        TalonFXConfiguration transConfig = new TalonFXConfiguration();

        CurrentLimitsConfigs transCurrentConfig = new CurrentLimitsConfigs();
        transCurrentConfig.withSupplyCurrentLimit(RobotMap.SwerveModule.TRANS_CURRENT_LIMIT);
        transCurrentConfig.withSupplyCurrentThreshold(RobotMap.SwerveModule.TRANS_THRESHOLD_CURRENT);
        transCurrentConfig.withSupplyTimeThreshold(RobotMap.SwerveModule.TRANS_THRESHOLD_TIME);
        transConfig.withCurrentLimits(transCurrentConfig);

        MotorOutputConfigs transOutputConfig = new MotorOutputConfigs();
        transOutputConfig.NeutralMode = NeutralModeValue.Brake;
        transConfig.withMotorOutput(transOutputConfig);

        Slot0Configs transPID = new Slot0Configs();
        transPID.kP = RobotMap.SwerveModule.TRANSLATION_KP;
        transPID.kI = RobotMap.SwerveModule.TRANSLATION_KI;
        transPID.kD = RobotMap.SwerveModule.TRANSLATION_KD;
        transConfig.withSlot0(transPID);

        translation.getConfigurator().apply(transConfig);

        translation.setInverted(RobotMap.SwerveModule.TRANSLATION_INVERTS[ID]);
    }

    private void configRotation() {
        rotation.clearStickyFaults();

        TalonFXConfiguration rotConfig = new TalonFXConfiguration();

        CurrentLimitsConfigs rotCurrentConfig = new CurrentLimitsConfigs();
        rotCurrentConfig.withSupplyCurrentLimit(RobotMap.SwerveModule.ROT_CURRENT_LIMIT);
        rotCurrentConfig.withSupplyCurrentThreshold(RobotMap.SwerveModule.ROT_THRESHOLD_CURRENT);
        rotCurrentConfig.withSupplyTimeThreshold(RobotMap.SwerveModule.ROT_THRESHOLD_TIME);
        rotConfig.withCurrentLimits(rotCurrentConfig);

        MotorOutputConfigs rotOutputConfig = new MotorOutputConfigs();
        rotOutputConfig.NeutralMode = NeutralModeValue.Brake;
        rotConfig.withMotorOutput(rotOutputConfig);

        Slot0Configs rotPID = new Slot0Configs();
        rotPID.kP = RobotMap.SwerveModule.ROTATION_KP;
        rotConfig.withSlot0(rotPID);

        rotation.getConfigurator().apply(rotConfig);

        rotation.setInverted(RobotMap.SwerveModule.ROTATION_INVERTS[ID]);
    }

    private void configCANcoder() {
        canCoder.clearStickyFaults();

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(RobotMap.SwerveModule.CAN_CODER_OFFSETS[ID]));
        canCoder.getConfigurator().apply(canCoderConfig);
    }
    /**
     * Sets translation and rotation motors to move to new state
     * @param state new state 
     */
    public void setAngleAndDrive(SwerveModuleState state) {
        state = optimize(state);

        VelocityDutyCycle transRequest = new VelocityDutyCycle(state.speedMetersPerSecond / RobotMap.SwerveModule.TRANS_ROT_TO_METERS);
        transRequest.withFeedForward(feedforward.calculate(state.speedMetersPerSecond)/RobotMap.MAX_VOLTAGE);
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
        rotation.getConfigurator().apply(new FeedbackConfigs().withFeedbackRotorOffset(position)); // rotations
        zeroTranslation();
    }

    public void zeroTranslation() {
        translation.getConfigurator().apply(new FeedbackConfigs().withFeedbackRotorOffset(0)); // rotations
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
    public SwerveModulePosition getSwerveModuleState() {
        return new SwerveModulePosition(getSpeed(), Rotation2d.fromDegrees(getAngle()));
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