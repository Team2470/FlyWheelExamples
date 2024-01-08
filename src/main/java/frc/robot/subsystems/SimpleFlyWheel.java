package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlyWheelConstants;

public class SimpleFlyWheel extends SubsystemBase {
    private final CANSparkMax m_leader;
    private final CANSparkMax m_follower;

    private final RelativeEncoder m_encoder;

    public SimpleFlyWheel() {
        // Create the leader motor
        m_leader = new CANSparkMax(FlyWheelConstants.kLeaderID, MotorType.kBrushless);

        // Create the follower motor
        m_follower = new CANSparkMax(FlyWheelConstants.KFollowerID, MotorType.kBrushless);

        // Configure the follower to follow the leader
        // sNeed to invert the follower as this motor is inverted (in this scenario) to the other motor
        m_follower.follow(m_leader, true);

        // Configure motor inversion. We want to ensure that positive values are meaningful.
        // Ie positive values mean game peice moves outwards
        // Also it is a good practice to explicty specify if the motor is inverted or not. One of the bring
        // up activies when testing of this code will be to verify that positive values are spin the flyweel 
        // the right way.
        m_leader.setInverted(false);
        m_follower.setInverted(false);

        // To reduce CAN Bus traffic we can turn down how often the Spark max sends data, that is not needed
        m_follower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        m_follower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        m_follower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

        // Setup current limit of 40 amps
        m_leader.setSmartCurrentLimit(40);
        m_follower.setSmartCurrentLimit(40);

        // Retrieve the built in motor encoder from the leader
        m_encoder = m_leader.getEncoder();
    }

    /**
     * FlyWheels current RPM
     * @return revolutions per second 
     */
    public double getVelocity() {
        // We are assuming a 1:1 gearing from the motor to flywheel
        return m_encoder.getVelocity();
    }

    /**
     * Periodic will run every 20ms reguardless if the robot is enabled/disabled.
     * This is a good place to put control logic for the system, and publish diagnostic info.
     */
    @Override
    public void periodic() {
        // Push data to the dashboard. The SmartDashboard is no longer maintained, but has been
        // replaced by ShuffleBoard and uses the same format. 
        SmartDashboard.putNumber("FlyWheel Velocity", getVelocity());
    }


    public void setOutputVoltage(double outputVoltage) {
        m_leader.setVoltage(outputVoltage);
    }

    public void stop() {
        m_leader.stopMotor();
    }

    public Command spinCommand(double outputVoltage) {
        return Commands.runEnd(
            // While this command is running set the flywheel to spin at 50% power
            ()-> this.setOutputVoltage(outputVoltage), 

            // Stop the flywheel motor, when the command ends (button is no longer pressed, or some other comamnd wants this subsystem)
            this::stop,

            // This command required this (the flywheel) subsystem
            this
        );
    }
}
