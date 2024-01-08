package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FlyWheelConstants;


public class LQRFlyWheel extends SubsystemBase {
    public enum Mode {
        kStopped,
        kOpenLoop,
        kClosedLoopPID,
        kClosedLoopLQR
    }

    //
    // Hardware
    //
    private final CANSparkMax m_leader;
    private final CANSparkMax m_follower;

    private final RelativeEncoder m_encoder;

    //
    // State
    //
    private Mode m_mode = Mode.kStopped;
    private double m_demand = 0.0;

    private final PIDController m_pidController = new PIDController(0, 0, 0);


    // The plant holds a state-space model of our flywheel. This system has the following properties:
    //
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.
    //
    // The Kv and Ka constants are found using the FRC Characterization toolsuite.
    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
        LinearSystemId.identifyVelocitySystem(FlyWheelConstants.kFlywheelKv, FlyWheelConstants.kFlywheelKa);

    // The observer fuses our encoder data and voltage inputs to reject noise.
    private final KalmanFilter<N1, N1, N1> m_observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            m_flywheelPlant,
            VecBuilder.fill(3.0), // How accurate we think our model is
            VecBuilder.fill(0.01), // How accurate we think our encoder
            // data is
            0.020);

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
        new LinearQuadraticRegulator<>(
            m_flywheelPlant,
            VecBuilder.fill(8.0), // Velocity error tolerance
            VecBuilder.fill(12.0), // Control effort (voltage) tolerance
            0.020);

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    private final LinearSystemLoop<N1, N1, N1> m_loop =
        new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 10.0, 0.020);

    // Create a new SysId routine for characterizing the flywheel.
    // This will be what will populate FlyWheelConstants.kFlywheelKv and FlyWheelConstants.kFlywheelKa with values
    private MutableMeasure<Voltage> m_sysIDOutputVolts = MutableMeasure.mutable(Volts.of(0));
    private MutableMeasure<Angle> m_sysIDRotations = MutableMeasure.mutable(Rotations.of(0));
    private MutableMeasure<Velocity<Angle>> m_sysIDVelocity = MutableMeasure.mutable(RPM.of(0)); 
    private final SysIdRoutine m_sysIDRoutine = new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            (Measure<Voltage> volts) -> {},//this.setOpenLoop(volts.in(Volts)),
            // Tell SysId how to record a frame of data for each motor on the mechanism being
            // characterized.
            log -> {
                // Record a fram for the flywheel motor
                log.motor("flywheel")
                    .voltage(m_sysIDOutputVolts)
                    .angularPosition(m_sysIDRotations)
                    .angularVelocity(m_sysIDVelocity);
            },
            // Tell SysId to make generated commands require this subsystem, suffix test state in
            // WPILog with this subsystem's name ("drive"),
            this
        )
    );


    public LQRFlyWheel() {
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

        // Compenstate for sensor reading latency. Try this code without it first!
        // For more info see: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#lqr-and-measurement-latency-compensation
        // m_controller.latencyCompensate(m_flywheelPlant, 0.02, 0.025);
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
     * Closed loop error. Difference between setpoint and current velocity.
     * @return closed cloosed error in RPM
     */
    public double getErrorRPM() {
        if (m_mode == Mode.kClosedLoopPID) {
            // getPositionError represents the delta/difference from the setpoint/goal velocity to current velocity
            // The naming of getPositionError is kind of misleading for this usecase, as we aren't using the PID controller
            // for position, but instead velocoty. There is a function called getVelocityError which will return how fast (or the rate :) ) the error is ranging
            return m_pidController.getPositionError();
        } else if (m_mode == Mode.kClosedLoopLQR) {
            return m_demand - m_encoder.getVelocity();
        }

        return 0;
    }

    /**
     * Closed loop percent error. 
     * 
     * At different speeds the allowable error in RPM varries. At slow speeds (like 400 RPM) a 100 RPM error is pretty significant,
     * however at fast speeds (3000 RPM) an error of 100 RPM is not as significant. 
     * Precent error can be used as a useful metric when waiting for the flywheel is near/at the goal speed.
     * https://www.calculator.net/percent-error-calculator.html
     * @return precent error. A value of 10 means 10%
     */
    public double getErrorPrecent(){
        if (m_mode == Mode.kClosedLoopPID || m_mode == Mode.kClosedLoopLQR) {
            return (m_demand - m_encoder.getVelocity() / m_demand) * 100;
        }

        return 0;
    }
    

    /**
     * When the robot enters auto or teleop reset the loop with the current encoder velocity
     */
    public void init() {
        m_loop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity())));
    }

    /**
     * Periodic will run every 20ms reguardless if the robot is enabled/disabled.
     * This is a good place to put control logic for the system, and publish diagnostic info.
     */
    @Override
    public void periodic() {
        double outputVoltage = 0.0;
       
        switch (m_mode) {
        case kOpenLoop:
            //
            // Open loop control
            //

            outputVoltage = m_demand;
            break;
        case kClosedLoopPID:
            //
            // PID Control
            //

            // Calculate feedforward output voltage
            outputVoltage = m_demand * FlyWheelConstants.kF; 

            // Update the PID controller, and its output to the feedforward voltage 
            outputVoltage += m_pidController.calculate(getVelocity());

            // Clamp output voltage to -10 to 10 volts
            outputVoltage = MathUtil.clamp(outputVoltage, -10, 10);

            // Send the command to the motor
            m_leader.setVoltage(outputVoltage);

            break;
        case kClosedLoopLQR:
            //
            // LQR StateSpace Control
            //

            // Set the next refence to our goal velocity
            // The LQR controller works in Radians/Second, not RPM so conversion is needed
            double goalRadS = Units.rotationsPerMinuteToRadiansPerSecond(m_demand);
            m_loop.setNextR(VecBuilder.fill(goalRadS));

            // Correct our Kalman filter's state vector estimate with encoder data.
            double currentVelocity = Units.rotationsPerMinuteToRadiansPerSecond(getVelocity());
            m_loop.correct(VecBuilder.fill(currentVelocity));

            // Update our LQR to generate new voltage commands and use the voltages to predict the next
            // state with out Kalman filter.
            m_loop.predict(0.020);

            // Send the new calculated voltage to the motors.
            // voltage = duty cycle * battery voltage, so
            // duty cycle = voltage / battery voltage
            outputVoltage = m_loop.getU(0);


            break;
        case kStopped:
        default:
            // If we are stopped, or don't know what mode we are in stop the motor. 
            outputVoltage = 0.0;
        }

        // Send the command to the motor
        m_leader.setVoltage(outputVoltage);

        //
        // SmartDashboard
        //
 
        // Push data to the dashboard. The SmartDashboard is no longer maintained, but has been
        // replaced by ShuffleBoard and uses the same format. 
        SmartDashboard.putNumber("FlyWheel Voltage", outputVoltage);
        SmartDashboard.putNumber("FlyWheel Velocity", getVelocity());
        SmartDashboard.putNumber("FlyWheel Goal Velocity", m_demand);
        SmartDashboard.putNumber("FlyWheel Error", getErrorRPM());
        SmartDashboard.putNumber("FlyWheel Error Precentage", getErrorPrecent());
        
        // Save it off for SysID
        m_sysIDOutputVolts.mut_replace(outputVoltage, Volts);
        m_sysIDRotations.mut_replace(m_encoder.getPosition(), Rotations);
    }

    public void stop() {
        m_mode = Mode.kStopped;
        m_demand  = 0.0;
    }

    public void setOpenLoop(double voltage) {
        m_mode = Mode.kOpenLoop;
        m_demand = voltage;
    }

    public Command setOpenLoopCommand(double voltage) {
        return Commands.runEnd(
            // While this command is running set the flywheel to spin at the given output voltage 
            ()-> this.setOpenLoop(voltage), 

            // Stop the flywheel motor, when the command ends (button is no longer pressed, or some other comamnd wants this subsystem)
            this::stop,

            // This command required this (the flywheel) subsystem
            this
        );
    }

    public void setClosedLoopPID(double goalRPM) {
        m_mode = Mode.kClosedLoopPID;
        m_demand = goalRPM;
    }

    public Command setClosedLoopPIDCommand(double goalRPM) {
        return Commands.runEnd(
            // While this command is running set the flywheel to spin at the given goal RPm 
            ()-> this.setClosedLoopPID(goalRPM), 

            // Stop the flywheel motor, when the command ends (button is no longer pressed, or some other comamnd wants this subsystem)
            this::stop,

            // This command required this (the flywheel) subsystem
            this
        );
    }

    public void setClosedLoopLQR(double goalRPM) {
        m_mode = Mode.kClosedLoopLQR;
        m_demand = goalRPM;
    }

    public Command setClosedLoopLQRCommand(double goalRPM) {
        return Commands.runEnd(
            // While this command is running set the flywheel to spin at the given goal RPm 
            ()-> this.setClosedLoopLQR(goalRPM), 

            // Stop the flywheel motor, when the command ends (button is no longer pressed, or some other comamnd wants this subsystem)
            this::stop,

            // This command required this (the flywheel) subsystem
            this
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIDRoutine.quasistatic(direction);
      }
    
      public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIDRoutine.dynamic(direction);
      }

}
