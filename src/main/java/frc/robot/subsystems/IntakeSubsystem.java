package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.operatorStuff;
//import frc.robot.Constants.IntakeShooter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Ultrasonic;
// import com.revrobotics.SparkAbsoluteEncoder.Type;
// import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.Constants.IntakeShooter;


public class IntakeSubsystem extends SubsystemBase{

    public static CANSparkMax m_intake = new CANSparkMax(operatorStuff.kIntake_ID, MotorType.kBrushless);
    public static final Ultrasonic m_sensHighIntake = new Ultrasonic(0, 1);
    public static final Ultrasonic m_sensLowIntake = new Ultrasonic(3,2);

    boolean isSensorHigh;
    boolean isSensorLow; 
    double lowValue; 
    double highValue;
    
    public IntakeSubsystem(){
        m_intake.setIdleMode(IdleMode.kBrake);
    }

    public void setIntakeSpeed(double speed){
        m_intake.set(speed);
        //SmartDashboard.putNumber("Intake speed", speed);
       }

       public void stopIntake(){
        m_intake.set(0);
       }

       public double getLowSensor(){
        return m_sensLowIntake.getRangeInches();
       }
       public double getHighSensor(){
        return m_sensHighIntake.getRangeInches();
       }


       // used to for detecting note 
        public void detectStage1(){
        lowValue = m_sensLowIntake.getRangeInches();
        if(lowValue < 13){
            m_intake.set(0);
            isSensorLow = true;
        }
        else 
            m_intake.set(IntakeShooter.kIntakeSpeed);
            isSensorLow = false; 
       }

        public void detectStage2(){
        highValue = m_sensHighIntake.getRangeInches();
        // change this condition 
        if(highValue < 13){
            m_intake.set(0);
            isSensorHigh = true;
        }
        else 
            m_intake.set(IntakeShooter.kIntakeSpeed);
            isSensorHigh = false; 
       }
       
  
       public void periodic(){
        SmartDashboard.putNumber("High Sensor reading", getHighSensor());
        SmartDashboard.putBoolean("High Sensor status", isSensorHigh);
        SmartDashboard.putBoolean("Low Sensor status", isSensorLow);
            }
       
        }