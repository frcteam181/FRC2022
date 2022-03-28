package frc.robot.commands.drive_train;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.*;

public class TurnToAngle extends CommandBase {

    DriveTrain m_driveTrain;

    double m_tolerance, m_arcLengthTicks;
    int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
    double m_count = 0;
    
    public TurnToAngle(DriveTrain driveTrain, double angle) {

        m_driveTrain = driveTrain;
        m_arcLengthTicks = angle * kEncoderTicksPerDegree;

        addRequirements(m_driveTrain);

    }

    @Override
    public void initialize() {
        
        //arclengthDegrees = m_arclengthEntry.getDouble(0);
        m_count = 0;
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
        
        m_driveTrain.teleopDrive(0, 0);
        m_driveTrain.motionMagicEndConfigTurn();

    }

    @Override
    public boolean isFinished() {
        
        return m_count >= STABLE_ITERATIONS_BEFORE_FINISHED;

    }
}
