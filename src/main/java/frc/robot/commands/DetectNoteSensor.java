package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;

// run motor until note is detected --> immediately stop motors 
public class DetectNoteSensor extends InstantCommand {
    private final IntakeSubsystem m_intake;
    private final double m_speed; 

    public DetectNoteSensor(IntakeSubsystem intake, double speed){
      m_intake = intake;
      m_speed = speed;  
      addRequirements(m_intake);
    }
    public void initialize(){
        m_intake.sensorDetection(m_speed);
    }
    
}