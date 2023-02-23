
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;


public class SwervePIDCmd extends CommandBase {
    private final SwerveSubsystem SwerveDrive;

    //proportion constant for turning PIDs
    private  double proportion = .008;
    //instantiate PID for turning motors
    private final PIDController turningMotor1PID = new PIDController(proportion, 0, 0);
    private final PIDController turningMotor2PID = new PIDController(proportion, 0, 0);
    private final PIDController turningMotor3PID = new PIDController(proportion, 0, 0);
    private final PIDController turningMotor4PID = new PIDController(proportion, 0, 0);

    //instantiate PID for speed motors
    private final PIDController SpeedMotor1PID = new PIDController(0, 0, 0);
    private final PIDController SpeedMotor2PID = new PIDController(0, 0, 0);
    private final PIDController SpeedMotor3PID = new PIDController(0, 0, 0);
    private final PIDController SpeedMotor4PID = new PIDController(0, 0, 0);


    public SwervePIDCmd(SwerveSubsystem SwerveDrive) {
        this.SwerveDrive = SwerveDrive;
        addRequirements(SwerveDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Double[] speeds = new Double[4];
        speeds[0] = -turningMotor1PID.calculate(SwerveDrive.turningPosRadToDegrees(SwerveDrive.turningEncoder1.getPosition()), 20);
        speeds[1] = -turningMotor2PID.calculate(SwerveDrive.turningPosRadToDegrees(SwerveDrive.turningEncoder2.getPosition()), 20);
        speeds[2] = -turningMotor3PID.calculate(SwerveDrive.turningPosRadToDegrees(SwerveDrive.turningEncoder3.getPosition()), 20);
        speeds[3] = -turningMotor4PID.calculate(SwerveDrive.turningPosRadToDegrees(SwerveDrive.turningEncoder4.getPosition()), 20);
        SwerveDrive.setMotors(speeds,0);
        }

    
    @Override
    public void end(boolean interrupted) {
        SwerveDrive.setMotors(new Double[4],0);
    }

    @Override
    public boolean isFinished() { 
        return false;
    }

    }
