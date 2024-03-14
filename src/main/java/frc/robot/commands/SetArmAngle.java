package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmAngle extends InstantCommand{
    private final ArmSubsystem m_arm; 
    private final double m_targetAngle; 
    
    public SetArmAngle(ArmSubsystem arm, double targetAngle){
      m_arm = arm;
      m_targetAngle = targetAngle;
      addRequirements(m_arm);

    }

    public void initialize(){
        m_arm.setShooterAngle(m_targetAngle);
    }
}