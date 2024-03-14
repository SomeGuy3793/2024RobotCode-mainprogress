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


  public class RightClimberSubsystem extends SubsystemBase{ 
    
    private CANSparkMax m_rightClimber = new CANSparkMax(operatorStuff.kClimberRight_ID, MotorType.kBrushless);
    
    RelativeEncoder climberEncoderRight=m_rightClimber.getEncoder(SparkRelativeEncoder.Type.kHallSensor,42);

  public RightClimberSubsystem(){
    
    m_rightClimber.setIdleMode(IdleMode.kBrake);

}

    

    public Command climbDownRight(){
      return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setRightClimber(-(operatorStuff.kClimberSpeed));
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
    }

    public Command climbUpRight(){
      return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setRightClimber(operatorStuff.kClimberSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
    }

    


    
      
    
    public void setRightClimber(double speed){
      m_rightClimber.set(-(speed));
    }

    
    
    public void stop(){
   
    m_rightClimber.set(0);
    }

    public void periodic(){
     
      SmartDashboard.getNumber("right climber encoder",climberEncoderRight.getPosition());
    };
    
}