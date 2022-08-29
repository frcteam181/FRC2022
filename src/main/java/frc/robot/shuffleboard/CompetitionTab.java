package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class CompetitionTab {

    private ShuffleboardTab m_competitionTab;
    private SendableChooser<Command> m_autoChooser;

    private Shooter m_shooter;

    public CompetitionTab(Shooter shooter) {

        ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition");
    
        // Auto Chooser
        m_autoChooser = new SendableChooser<Command>();

        // Used Subsystem
        m_shooter = shooter;
    
        // Sets Default Auto
        //m_autoChooser.setDefaultOption("Taxi Only", new TaxiOnly(m_driveTrain));
        
        // Add autos here:
        //m_autoChooser.addOption("Taxi Only", new TaxiOnly(m_driveTrain));
    
        m_competitionTab.add("Choose Auto", m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(4, 3).withSize(2, 1);
        m_competitionTab.addNumber("Shooter RPM", m_shooter::getRPM).withPosition(0, 0);
        

    }

    public Command getAuto() {
        return m_autoChooser.getSelected();
    }
    
}
