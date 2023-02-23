package frc.robot.commands;

//timer library
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;


public class DriveForwardCmd extends CommandBase {
      //declare startTime variable
        private double startTime;
        private final SwerveSubsystem SwerveDrive;
        private final double distance;
    
    public DriveForwardCmd(SwerveSubsystem SwerveDrive, double distance) {
        this.SwerveDrive = SwerveDrive;
        //this.distance = SwerveDrive.getEncoderMeters() + distance;
        this.distance = 0;
        addRequirements(SwerveDrive);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  
        //set starttime to time when autonomous mode is started
        startTime = Timer.getFPGATimestamp();

        //Set turning encoders to 0
        SwerveDrive.turningEncoder1.setPosition(0);
        SwerveDrive.turningEncoder2.setPosition(0);
        SwerveDrive.turningEncoder3.setPosition(0);
        SwerveDrive.turningEncoder4.setPosition(0);

        //put motors in brake mode
        SwerveDrive.enableMotors(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //set current time to time since robot turned on
    double time = Timer.getFPGATimestamp();


    //run speed motor at 10% speed for 3 seconds
    if (time - startTime < 3) {
        SwerveDrive.setMotors(new Double[4],0.1);
    }
    else
        SwerveDrive.setMotors(new Double[4],0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
      
}
