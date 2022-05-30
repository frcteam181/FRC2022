package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase{
    
    private CANSparkMax m_sparkShooter;

    private SparkMaxPIDController m_shooterPID;

    private double m_nearSpeed, m_farSpeed;

    private boolean m_isFarTarget, m_isShooterEnabled;

    private RelativeEncoder m_shooterEncoder;

    public Shooter() {

        m_sparkShooter = new CANSparkMax(kSHOOTER, MotorType.kBrushless);

        m_sparkShooter.restoreFactoryDefaults();

        m_shooterPID = m_sparkShooter.getPIDController();

        m_shooterPID.setP(kShooterGains.kP);
        m_shooterPID.setI(kShooterGains.kI);
        m_shooterPID.setD(kShooterGains.kD);
        m_shooterPID.setFF(kShooterGains.kF);
        m_shooterPID.setIZone(kShooterGains.kIzone);
        m_shooterPID.setOutputRange(0, kShooterGains.kPeakOutput);
        
        m_shooterEncoder = m_sparkShooter.getEncoder();

        m_nearSpeed = 4000; // rpm
        m_farSpeed = 2000; // rpm

        m_isFarTarget = false;

        stop();

    }

    public void shoot(double rpm) {
        m_shooterPID.setReference(rpm, ControlType.kSmartVelocity);
    }

    public void nearShot() {

        //m_shooterPID.setReference(m_nearSpeed, ControlType.kVelocity);
        m_sparkShooter.set(0.8);
        m_isFarTarget = false;

    }

    public void farShot() {
        m_shooterPID.setReference(m_farSpeed, ControlType.kVelocity);
        m_isFarTarget = true;
    }

    public void stop() {
        m_sparkShooter.set(0);
    }

    public double getRPM() {
        return m_shooterEncoder.getVelocity();
    }

    public void switchShooterState() {
        m_isShooterEnabled = !m_isShooterEnabled;
    }

}
