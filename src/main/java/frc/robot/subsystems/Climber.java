package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;

public class Climber extends SubsystemBase {

    private CANSparkMax m_leftSpool, m_rightSpool;
    private RelativeEncoder m_leftEncoder;
    private SparkMaxPIDController m_leftPID;
    private DoubleSolenoid m_leftPiston, m_rightPiston;
    private boolean m_isClimberUp;
    private double m_deadband = 0.1;

    public Climber() {

        // Spools
        m_leftSpool = new CANSparkMax(kCLIMBER_LEFT, MotorType.kBrushless);
        m_rightSpool = new CANSparkMax(kCLIMBER_RIGHT, MotorType.kBrushless);

        m_leftSpool.restoreFactoryDefaults();
        m_rightSpool.restoreFactoryDefaults();

        m_leftSpool.setIdleMode(IdleMode.kBrake);

        // Encoder
        m_leftEncoder = m_leftSpool.getEncoder();
        m_leftEncoder.setPosition(0); // ??? We need a better way to zero its position !!!

        // PID
        m_leftPID = m_leftSpool.getPIDController();
        m_leftPID.setP(kClimberGains.kP, kCLIMBER_PID_SLOT);
        m_leftPID.setI(kClimberGains.kI, kCLIMBER_PID_SLOT);
        m_leftPID.setD(kClimberGains.kD, kCLIMBER_PID_SLOT);
        m_leftPID.setFF(kClimberGains.kF, kCLIMBER_PID_SLOT);
        m_leftPID.setIZone(kClimberGains.kIzone, kCLIMBER_PID_SLOT);
        m_leftPID.setOutputRange(-kClimberGains.kPeakOutput, kClimberGains.kPeakOutput, kCLIMBER_PID_SLOT);

        // Follower Spool
        m_rightSpool.follow(m_leftSpool, true);

        // Pistons
        m_leftPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
        m_rightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);

        m_isClimberUp = true;

    }

    public void tiltClimber() {

        m_isClimberUp = !m_isClimberUp;
        if (m_isClimberUp) {
            m_leftPiston.set(Value.kForward);
            m_rightPiston.set(Value.kForward);
        } else {
            m_leftPiston.set(Value.kReverse);
            m_rightPiston.set(Value.kReverse);
        }

    }

    public void moveClimber(double speed) {
        m_leftSpool.set(deadband(speed));
    }

    public void autoMoveClimber(double position) {
        m_leftPID.setReference(position, ControlType.kPosition);
    }

    public void resetEncoder() {
        m_leftEncoder.setPosition(0);
    }

    public double deadband(double value) {

        /** Upped Deadband */
        if (value >= m_deadband)
            return value; // kBaseSpeed;
        /** Lower Deadband */
        if (value <= -m_deadband)
            return value;
        /** Inside Deadband */
        return 0;

    }

}
