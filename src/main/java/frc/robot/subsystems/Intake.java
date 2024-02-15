package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private CANSparkMax roller;
    private CANSparkMax deploy;

    private SparkPIDController deployPositionPID;

    private RelativeEncoder deployEncoder;

    private DigitalInput limitSwitch;

    private Intake() {
        roller = new CANSparkMax(RobotMap.Intake.ROLLER_ID, MotorType.kBrushless);
        deploy = new CANSparkMax(RobotMap.Intake.DEPLOY_ID, MotorType.kBrushless);

        limitSwitch = new DigitalInput(RobotMap.Intake.LIMIT_SWITCH_ID);

        deployPositionPID = deploy.getPIDController();
        deployEncoder = deploy.getEncoder();

        roller.setInverted(RobotMap.Intake.ROLLER_INVERT);
        deploy.setInverted(RobotMap.Intake.DEPLOY_INVERT);

        configMotors();
    }

    public void configMotors() {
        roller.restoreFactoryDefaults();
        deploy.restoreFactoryDefaults();

        roller.setSmartCurrentLimit(RobotMap.Intake.ROLLER_CURRENT_LIMIT);
        roller.setIdleMode(IdleMode.kCoast);

        deploy.setIdleMode(IdleMode.kBrake);
        deployPositionPID.setP(RobotMap.Intake.DEPLOY_kP);

        roller.burnFlash();
        deploy.burnFlash();
    }

    public void setSensorPosition(double pos) {
        deployEncoder.setPosition(pos);
    }

    public void setDeployPos(double rots) {
        deployPositionPID.setReference(rots, ControlType.kPosition);
    }

    public boolean limitSwitchHit() {
        return limitSwitch.get();
    }

    public void setDeployPower(double power) {
        deploy.set(power);
    }

    public void setRollerPower(double power) {
        roller.setVoltage(power * RobotMap.MAX_VOLTAGE);
    }
    
    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }
}
