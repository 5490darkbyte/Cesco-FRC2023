package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.RobotContainer;

public class SwerveControllerCmd extends CommandBase{
    private final SwerveSubsystem SwerveDrive;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SwerveControllerCmd(SwerveSubsystem SwerveDrive) {
      this.SwerveDrive = SwerveDrive;
        // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(SwerveDrive);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //slow speed down to 70%
        //Negative to make forward positive
        double speed = -RobotContainer.driverController.getRawAxis(1) * .07;
        //slow turning down to 70%
        //Negative to make right positive
        double turn = -RobotContainer.driverController.getRawAxis(4) * .07;

        // deadband for if joystick reading is slightly off from zero
        if (Math.abs(speed) < 0.05) {
        speed = 0;
        }
        if (Math.abs(turn) < 0.05) {
        turn = 0;
        }
        
        Double[] turnArray = {turn,turn,turn,turn};
         SwerveDrive.setMotors(turnArray, speed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //put motors in coast mode
        SwerveDrive.enableMotors(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    
}
