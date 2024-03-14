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

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class IntakeSubsystem extends SubsystemBase{

    public static CANSparkMax m_intake = new CANSparkMax(operatorStuff.kIntake_ID, MotorType.kBrushless);
    public static final Ultrasonic m_sensHighIntake = new Ultrasonic(0, 1);
    public static final Ultrasonic m_sensLowIntake = new Ultrasonic(3,2);
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kNoteColor = new Color(0.52, 0.36, 0.09);

    boolean isSensorHigh;
    boolean isSensorLow; 
    double lowValue; 
    double colorAccuracy;
    
    public IntakeSubsystem(){
        m_intake.setIdleMode(IdleMode.kBrake);
        Ultrasonic.setAutomaticMode(true);
        m_colorMatcher.addColorMatch(kNoteColor);
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
       
       // color detection 
       public void detectColor(){
        Color detectedColor = m_colorSensor.getColor();
        String colorString; 
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        if(match.color == kNoteColor){
            colorString = "Orange Note";
        }
        else{
            colorString = "Unknown";
        }
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);
       }

       // used for detecting note 
        public void sensorDetection(double speed){
        lowValue = m_sensLowIntake.getRangeInches();
        if(lowValue < 6){
            m_intake.set(0);
            isSensorLow = true;
        }
        else 
            m_intake.set(speed);
            isSensorLow = false; 
       }
       
       public void colorDetection(double speed){
        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        while(match.confidence > 92){
            m_intake.set(0);
        }
        
       }

       @Override
        public void periodic(){
        detectColor();
        SmartDashboard.putNumber("High Sensor reading", getHighSensor());
        SmartDashboard.putNumber("Low Sensor reading", getLowSensor());
        SmartDashboard.putBoolean("High Sensor status", isSensorHigh);
        SmartDashboard.putBoolean("Low Sensor status", isSensorLow);
            }
       
        }