package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Intake extends SubsystemBase{

    private CANSparkMax m_intakeMotor;
    private SparkMaxPIDController m_intakePID;

    private DoubleSolenoid m_intakePiston;
    private boolean m_isIntakeUp;
    
    public Intake() {

        // SparkMax
        m_intakeMotor = new CANSparkMax(kINTAKE, MotorType.kBrushless);
        m_intakeMotor.restoreFactoryDefaults();
        m_intakeMotor.setInverted(true);
        m_intakePID = m_intakeMotor.getPIDController();
        m_intakePID.setP(kIntakeGains.kP);
        m_intakePID.setI(kIntakeGains.kI);
        m_intakePID.setD(kIntakeGains.kD);
        m_intakePID.setFF(kIntakeGains.kF);
        m_intakePID.setIZone(kIntakeGains.kIzone);
        m_intakePID.setOutputRange(-1, 1);

        // Double Solenoid
        m_intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
        m_isIntakeUp = false;
        switchIntake();

    }

    public void switchIntake() {
        m_isIntakeUp = !m_isIntakeUp;
        if (m_isIntakeUp) {
            m_intakePiston.set(DoubleSolenoid.Value.kReverse);
        } else {
            m_intakePiston.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void moveIntakeDown() {
        m_isIntakeUp = false;
        m_intakePiston.set(DoubleSolenoid.Value.kForward);
    }

    public void collectBalls(){
        m_intakeMotor.set(kIntakeSpeed);
    }

    public void ejectBalls() {
        m_intakeMotor.set(-kIntakeSpeed);
    }

    public void stop() {
        m_intakeMotor.set(0);
    }
    
}
