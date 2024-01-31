package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.subsystems.swerve.Drivetrain;

public class Telemetry {
    private NetworkTable table;
    private NetworkTableInstance inst;

    private Drivetrain drive = Drivetrain.getInstance();
    StructArrayPublisher<SwerveModuleState> swervePublisher;

    public Telemetry() {
        inst = NetworkTableInstance.getDefault();
        // table = inst.getTable("S");
    }

    public void swerve() {
        table = inst.getTable("Swerve");
        NetworkTable _states = table.getSubTable("_states");

        swervePublisher = NetworkTableInstance.getDefault().getTable("Swerve")
            .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    }

    public void publish() {
        swervePublisher.set(drive.getModuleStates());
    }
}