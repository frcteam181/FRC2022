package frc.robot.commands.drive_train;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.*;

public class DriveMMTest extends CommandBase{

    DriveTrain m_driveTrain;
    ShuffleboardTab m_driveMMTab;
    double m_targetPosition;
    double m_targetTicks;
    int m_count;
    double m_startTime;
    double m_driveKp, m_driveKi, m_driveKd, m_driveKf;
    NetworkTableEntry m_kpEntry, m_kiEntry, m_kdEntry, m_kfEntry, m_targetPosEntry, m_targetTicksEntry, m_iterationEntry, m_driveDurationEntry;
    int STABLE_ITERATIONS_BEFORE_FINISHED = 5;

    public DriveMMTest(DriveTrain driveTrain, double targetInches) {

        m_driveTrain = driveTrain;
        m_targetPosition = targetInches;

        NetworkTable m_driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Testing");

        m_kpEntry = m_driveTab.getEntry("kP");
        m_kiEntry = m_driveTab.getEntry("kI");
        m_kdEntry = m_driveTab.getEntry("kD");
        m_kfEntry = m_driveTab.getEntry("kF");
        m_iterationEntry = m_driveTab.getEntry("Finish Iterations");
        m_targetPosEntry = m_driveTab.getEntry("Tgt. Inches");
        m_targetTicksEntry = m_driveTab.getEntry("Tgt. Ticks");
        m_driveDurationEntry = m_driveTab.getEntry("Run Time");

        addRequirements(m_driveTrain);

    }

    @Override
    public void initialize() {
    
        STABLE_ITERATIONS_BEFORE_FINISHED = (int) m_iterationEntry.getDouble(5.0);
        m_targetTicks = m_targetPosEntry.getDouble(0) * kEncoderTicksPerInch;
        m_targetTicksEntry.forceSetDouble((int) m_targetTicks);
        m_count = 0;
        m_driveTrain.resetDrivePIDValues(m_kpEntry.getDouble(0.0), m_kiEntry.getDouble(0.0), m_kfEntry.getDouble(0), m_kfEntry.getDouble(0));
        m_driveTrain.motionMagicStartConfigDrive(m_targetTicks >= 0, m_targetTicks);
    }

    @Override
    public void execute() {
        
        if (m_driveTrain.motionMagicDrive(m_targetTicks)) {
            m_count++;
        } else {
            m_count = 0;
        }

    }

    @Override
    public void end(boolean interrupted) {
        
        m_driveTrain.teleopDrive(0, 0);

    }

    @Override
    public boolean isFinished() {
        
        return m_count >= STABLE_ITERATIONS_BEFORE_FINISHED;

    }
    
}
