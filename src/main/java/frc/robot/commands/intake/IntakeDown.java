package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeDown extends CommandBase{

    Intake m_intake;

    public IntakeDown(Intake intake) {

        m_intake = intake;

        addRequirements(intake);

    }

    @Override
    public void initialize() {
        m_intake.intakeDown();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}