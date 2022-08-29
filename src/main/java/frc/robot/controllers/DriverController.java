package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.default_commands.DriveTrainDefaultCommand;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.*;

public class DriverController {

    private Joystick m_controller;

    private JoystickButton m_tg;

    private POVButton m_up, m_dw, m_l, m_r;

    // Used Robot Subsystems //
    private DriveTrain m_driveTrain;

    public DriverController(DriveTrain driveTrain) {

        m_controller = new Joystick(kDRIVER_CONTROLLER);
        
        // Used Subsystems Instances //
        m_driveTrain = driveTrain;

        // Xbox Controller Buttons
        m_tg = new JoystickButton(m_controller, ButtonType.kTrigger.value);
        m_up = new POVButton(m_controller, 0);
        m_r = new POVButton(m_controller, 90);
        m_dw = new POVButton(m_controller, 180);
        m_l = new POVButton(m_controller, 270);

        // Default Command
        m_driveTrain.setDefaultCommand(new DriveTrainDefaultCommand(m_driveTrain, m_controller));

        bindButtons();

    }

    public void bindButtons() {

    }
    
}
