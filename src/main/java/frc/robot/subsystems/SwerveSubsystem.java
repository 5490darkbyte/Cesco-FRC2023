package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;


//motor controller libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//turning encoder library
import com.ctre.phoenix.sensors.CANCoder;



public class SwerveSubsystem extends SubsystemBase {
//instantiate turning motor controllers and speed motor controllers 
  private final CANSparkMax turningMotor1 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax speedMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax turningMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax speedMotor2 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax turningMotor3 = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax speedMotor3 = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax turningMotor4 = new CANSparkMax(8, MotorType.kBrushless);
  private final CANSparkMax speedMotor4 = new CANSparkMax(7, MotorType.kBrushless);
  
  
    //instantiate turning encoders 
    public final CANCoder turningEncoder1 = new CANCoder(13);
    public final CANCoder turningEncoder2 = new CANCoder(14);
    public final CANCoder turningEncoder3 = new CANCoder(15);
    public final CANCoder turningEncoder4 = new CANCoder(16);




   /** Creates a new SwerveSubsystem. */
   public SwerveSubsystem() {
    //Make right positive for all turning encoders
    turningEncoder1.configSensorDirection(true);
    turningEncoder2.configSensorDirection(true);
    turningEncoder3.configSensorDirection(true);
    turningEncoder4.configSensorDirection(true);

    //Set turning encoders to magnet absolute
    turningEncoder1.setPositionToAbsolute();
    turningEncoder2.setPositionToAbsolute();
    turningEncoder3.setPositionToAbsolute();
    turningEncoder4.setPositionToAbsolute();
   }


    /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        //Display Motor Speeds. Value between -1.0 and 1.0
        SmartDashboard.putNumber("Turning Motor1 Speed", turningMotor1.get());
        SmartDashboard.putNumber("Speed Motor1 Speed", speedMotor1.get());
    
        SmartDashboard.putNumber("Turning Motor2 Speed", turningMotor2.get());
        SmartDashboard.putNumber("Speed Motor2 Speed", speedMotor2.get());
    
        SmartDashboard.putNumber("Turning Motor3 Speed", turningMotor3.get());
        SmartDashboard.putNumber("Speed Motor3 Speed", speedMotor3.get());
    
        SmartDashboard.putNumber("Turning Motor4 Speed", turningMotor4.get());
        SmartDashboard.putNumber("Speed Motor4 Speed", speedMotor4.get());
    
        //Display encoder positions in radians and degrees
        SmartDashboard.putNumber("Turning Encoder1 Radians", turningEncoder1.getPosition());
        SmartDashboard.putNumber("Turning Encoder1 Degrees", turningPosRadToDegrees(turningEncoder1.getPosition()));
        SmartDashboard.putNumber("Turning Encoder2 Radians", turningEncoder2.getPosition());
        SmartDashboard.putNumber("Turning Encoder2 Degrees", turningPosRadToDegrees(turningEncoder2.getPosition()));
        SmartDashboard.putNumber("Turning Encoder3 Radians", turningEncoder3.getPosition());
        SmartDashboard.putNumber("Turning Encoder3 Degrees", turningPosRadToDegrees(turningEncoder3.getPosition()));
        SmartDashboard.putNumber("Turning Encoder4 Radians", turningEncoder4.getPosition());
        SmartDashboard.putNumber("Turning Encoder4 Degrees", turningPosRadToDegrees(turningEncoder4.getPosition()));

        
        //Display Xbox Axis Positions. Values from -1.0 to 1.0
        SmartDashboard.putNumber("RawAxis A", -RobotContainer.driverController.getRawAxis(1));
        SmartDashboard.putNumber("RawAxis B", -RobotContainer.driverController.getRawAxis(4));
  }

  
  public double drivingPosToFeet(double measured) {
    //PPR, gear ratio, radius in inches
    return measured / 4096.0 * 1 / 8.14 * 2 * Math.PI * 2;
  }

  public double turningPosRadToDegrees(double measured){
    return  (measured * 180/Math.PI);
  } 

    //set motor to brake mode or coast mode
    public void enableMotors(boolean on) {
        IdleMode mode;
        if (on) {
          mode = IdleMode.kBrake;
        } else {
          mode = IdleMode.kCoast;
        }
        turningMotor1.setIdleMode(mode);
        speedMotor1.setIdleMode(mode);
        turningMotor2.setIdleMode(mode);
        speedMotor2.setIdleMode(mode);
        turningMotor3.setIdleMode(mode);
        speedMotor3.setIdleMode(mode);
        turningMotor4.setIdleMode(mode);
        speedMotor4.setIdleMode(mode);
      }


      public void setMotors(Double[] desiredSpeeds, double speed) {
        //next we tell the motor controller how fast the motor should spin 
        //input current encoder degree position and goal degree position to PID
        speedMotor1.set(speed);
        turningMotor1.set(desiredSpeeds[0]);
        speedMotor2.set(speed);
        turningMotor2.set(desiredSpeeds[1]);
        speedMotor3.set(speed);
        turningMotor3.set(desiredSpeeds[2]);
        speedMotor4.set(speed);
        turningMotor4.set(desiredSpeeds[3]);
    }


}
