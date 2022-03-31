package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeUp extends CommandBase{

    Intake m_intake;

    public IntakeUp(Intake intake) {

        m_intake = intake;

        addRequirements(intake);

    }

    @Override
    public void initialize() {
        
        m_intake.intakeUp();

    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}