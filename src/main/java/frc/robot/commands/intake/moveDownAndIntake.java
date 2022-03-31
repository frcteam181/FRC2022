package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class moveDownAndIntake extends CommandBase{

    Intake m_intake;

    public moveDownAndIntake(Intake intake) {

        m_intake = intake;

        addRequirements(intake);

    }

    @Override
    public void execute() {
        
        m_intake.intakeDown();
        m_intake.collectBalls();

    }

    @Override
    public void end(boolean interrupted) {
        m_intake.intakeUp();
        m_intake.stop();
    }
    
}