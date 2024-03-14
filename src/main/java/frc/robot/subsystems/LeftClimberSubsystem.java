package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.desiredEncoderValue;
import frc.robot.Constants.operatorStuff;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


  public class LeftClimberSubsystem extends SubsystemBase{ 
    private CANSparkMax m_leftClimber = new CANSparkMax(operatorStuff.kClimberLeft_ID, MotorType.kBrushless);
   
    RelativeEncoder climberEncoderLeft=m_leftClimber.getEncoder(SparkRelativeEncoder.Type.kHallSensor,42);
    

  public LeftClimberSubsystem(){
    m_leftClimber.setIdleMode(IdleMode.kBrake);
   

}

  

   

   
    

    public Command climbDownLeft(){
      return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setLeftClimber(-(operatorStuff.kClimberSpeed));
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
    }


    public Command climbUpLeft(){
      return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setLeftClimber(operatorStuff.kClimberSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
    }
      
    
   
    public void setLeftClimber(double speed){
        m_leftClimber.set(speed);
    }
    
    public void stop(){
    m_leftClimber.set(0);
   
    }

    public void periodic(){
      SmartDashboard.getNumber("left climber encoder",climberEncoderLeft.getPosition());
      
    };
    
}