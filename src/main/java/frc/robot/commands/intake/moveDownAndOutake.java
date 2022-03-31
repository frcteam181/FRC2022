package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class moveDownAndOutake extends CommandBase{

    Intake m_intake;

    public moveDownAndOutake(Intake intake) {

        m_intake = intake;

        addRequirements(intake);

    }

    @Override
    public void execute() {
        
        m_intake.intakeDown();
        m_intake.ejectBalls();

    }

    @Override
    public void end(boolean interrupted) {
        m_intake.intakeUp();
        m_intake.stop();
    }
    
}