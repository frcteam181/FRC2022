package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;

public class Shooter {
    
    CANSparkMax m_sparkShooter;

    SparkMaxPIDController m_shooterPID;

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

    }

    public void shoot(double rpm) {

        m_shooterPID.setReference(rpm, ControlType.kSmartVelocity);

    }

    public void stop() {

        m_shooterPID.setReference(0, ControlType.kSmartVelocity);

    }

}
