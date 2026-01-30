package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

public class NewTurretCommand extends Command {

    // The turret subsystem we will control
    private final TurretSubsystem myturret;
    
    private final CommandSwerveDrivetrain m_Drivetrain;

    // PID controller that decides how fast the turret should rotate
    private static final PIDControllerConfigurable rotationalPidController =
        new PIDControllerConfigurable(0.05000, 0.000000, 0.000000, 1.0);

    // The field direction (in degrees) we want the turret to always face
    private double desiredFieldAngle = 45;

    // Constructor: we receive the turret subsystem and the Pigeon gyro
    public NewTurretCommand(TurretSubsystem turret, CommandSwerveDrivetrain drivetrain) {
        this.myturret = turret;   // store the turret subsystem
        this.m_Drivetrain = drivetrain;
        addRequirements(myturret, m_Drivetrain);   // tell WPILib we control the turret 
    }

    @Override
    public void execute() {

        // STEP 1: Read the robot's current yaw angle from the Pigeon
        // This tells us how much the robot has rotated on the field
        double robotYaw = m_Drivetrain.getPigeon2().getYaw().getValueAsDouble();
        SmartDashboard.putNumber("Ttest/RobotAngle", robotYaw);

        // STEP 2: Read the turret's current angle from its encoder
        // This tells us where the turret is pointing relative to the robot
        double turretAngle = myturret.getAngle();
        turretAngle = Math.IEEEremainder(turretAngle, 360);
        SmartDashboard.putNumber("Ttest/TurretAngle", turretAngle);

        // STEP 3: Compute how far off we are from the desired field direction
        // We combine robotYaw + turretAngle to get the turret's field angle
        // Then subtract from the desired angle to get the error
        double error = desiredFieldAngle - (turretAngle + robotYaw);
                error = Math.IEEEremainder(error, 360.0);

        SmartDashboard.putNumber("Ttest/desiredAngle" , desiredFieldAngle);
        SmartDashboard.putNumber("Ttest/Error", error);
        // STEP 4: Feed the error into the PID controller
        // The PID returns how fast the turret should rotate to fix the error
        double output = rotationalPidController.calculate(error, 1);
        SmartDashboard.putNumber("Ttest/PIDOutput", output);
        // STEP 5: Send the PID output to the turret motor
        // Positive or negative speed depending on which way we need to turn
        myturret.setMotor(output);
        
    }

    @Override
    public boolean isFinished() {
        // This command runs forever until interrupted
        return false;
    }
}
