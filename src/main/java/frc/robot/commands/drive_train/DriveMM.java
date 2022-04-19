package frc.robot.commands.drive_train;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.*;

public class DriveMM extends CommandBase {

    DriveTrain m_driveTrain;

    double m_targetPosition;
    int m_count, STABLE_ITERATIONS_BEFORE_FINISHED = 5;

    public DriveMM(DriveTrain driveTrain, double targetInches) {

        m_driveTrain = driveTrain;

        // Distance in inches to ticks conversion is:
        // WheelRotations = positionMeters/(2 * Math.PI * kWheelRadiusInches);
	   	// motorRotations = wheelRotations * kSensorGearRatio
		// sensorCounts =   motorRotations * kCountsPerRev
        m_targetPosition = targetInches * kEncoderTicksPerInch;

        addRequirements(m_driveTrain);

    }

    @Override
    public void initialize() {

        m_driveTrain.motionMagicStartConfigDrive( m_targetPosition);

    }

    @Override
    public void execute() {

        if (m_driveTrain.motionMagicDrive(m_targetPosition)) {
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
