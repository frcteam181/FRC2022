package frc.robot.commands.drive_train;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.*;

public class TurnToAngleTest extends CommandBase {

    DriveTrain m_driveTrain;

    public static double m_currentAngle;
    double m_targetAngle, m_tolerance, m_speed, m_arcLengthTicks;
    int m_count;
    private double m_startTime, m_turnKp, m_turnKi, m_turnKd, m_turnKf;
    private NetworkTableEntry m_turnKpEntry, m_turnKiEntry, m_turnKdEntry, m_turnKfEntry, m_arcLengthEntry, m_iterationEntry, m_driveDurationEntry, m_arcLengthTicksEntry;
    int STABLE_ITERATIONS_BEFORE_FINISHED = 5;

    public TurnToAngleTest(DriveTrain driveTrain, double angle) {

        m_driveTrain = driveTrain;
        m_targetAngle = angle;
        NetworkTable m_driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Testing");
        m_turnKpEntry = m_driveTab.getEntry("kP");
        m_turnKiEntry = m_driveTab.getEntry("kI");
        m_turnKdEntry = m_driveTab.getEntry("kD");
        m_turnKfEntry = m_driveTab.getEntry("kF");
        m_iterationEntry = m_driveTab.getEntry("Finish Iterations");
        m_arcLengthEntry = m_driveTab.getEntry("Tgt. Degrees");
        m_arcLengthTicksEntry = m_driveTab.getEntry("Tgt. Ticks");
        m_driveDurationEntry = m_driveTab.getEntry("Run Time");

        addRequirements(m_driveTrain);

    }

    @Override
    public void initialize() {
        
        m_arcLengthTicks = m_arcLengthEntry.getDouble(0) * kEncoderTicksPerDegree;
        m_arcLengthTicksEntry.setDouble(m_arcLengthTicks);
        m_turnKp = m_turnKpEntry.getDouble(0.0);
        m_turnKi = m_turnKiEntry.getDouble(0.0);
        m_turnKd = m_turnKdEntry.getDouble(0.0);
        m_turnKf = m_turnKfEntry.getDouble(0.0);
        STABLE_ITERATIONS_BEFORE_FINISHED = (int) m_iterationEntry.getDouble(5.0);
        m_startTime = Timer.getFPGATimestamp();
        m_count = 0;
        m_driveTrain.resetTurnPIDValues(m_turnKp, m_turnKi, m_turnKd, m_turnKf);
        m_driveTrain.motionMagicStartConfigsTurn((m_arcLengthTicks < 0), m_arcLengthTicks);

    }

    @Override
    public void execute() {
        
        if (m_driveTrain.motionMagicTurn(m_arcLengthTicks)) {
            m_count++;
        } else {
            m_count = 0;
        }

    }

    @Override
    public void end(boolean interrupted) {
        
        double m_driveDuration = Timer.getFPGATimestamp() - m_startTime;
        m_driveDurationEntry.setDouble(m_driveDuration);
        m_driveTrain.teleopDrive(0, 0);
        m_driveTrain.motionMagicEndConfigTurn();

    }

    @Override
    public boolean isFinished() {
        
        return m_count >= STABLE_ITERATIONS_BEFORE_FINISHED;

    }
}
