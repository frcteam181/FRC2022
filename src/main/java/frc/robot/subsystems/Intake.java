package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Intake extends SubsystemBase{

    private CANSparkMax m_intakeMotor;

    private SparkMaxPIDController m_intakePID;

    private DigitalInput m_intakeSoftStop;

    private DoubleSolenoid m_intakePiston;
    
    public Intake() {

        m_intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

        m_intakeMotor = new CANSparkMax(kINTAKE, MotorType.kBrushless);

        m_intakeMotor.restoreFactoryDefaults();

        m_intakePID = m_intakeMotor.getPIDController();

        m_intakePID.setP(kIntakeGains.kP);
        m_intakePID.setI(kIntakeGains.kI);
        m_intakePID.setD(kIntakeGains.kD);
        m_intakePID.setFF(kIntakeGains.kF);
        m_intakePID.setIZone(kIntakeGains.kIzone);
        m_intakePID.setOutputRange(-1, 1);

        m_intakeSoftStop = new DigitalInput(0);

        intakeUp();

    }

    public void intakeDown() {
        m_intakePiston.set(DoubleSolenoid.Value.kForward);
    }

    public void intakeUp() {
        m_intakePiston.set(DoubleSolenoid.Value.kReverse);
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

    public boolean isIntakeHome() {
        return m_intakeSoftStop.get();
    }

    public boolean isIntakeUp() {
        if (m_intakePiston.get() == Value.kForward) {
            return false;
        }else {
            return true;
        }
    }

    public void move_piston() {
        if (isIntakeUp() == true) {
            m_intakePiston.set(DoubleSolenoid.Value.kForward);
        } else {
            m_intakePiston.set(DoubleSolenoid.Value.kReverse);
        }
    }
    
}
