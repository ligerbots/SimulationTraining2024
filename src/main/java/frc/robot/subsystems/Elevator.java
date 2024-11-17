// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements AutoCloseable {
    public static final double HEIGHT_TOLERANCE = 0.0254; // meters = 1 inch

    static final int kMotorPort = 0;
    static final int kEncoderAChannel = 0;
    static final int kEncoderBChannel = 1;
    static final int kJoystickPort = 0;

    static final double kElevatorKp = 5;
    static final double kElevatorKi = 0;
    static final double kElevatorKd = 0;

    static final double kElevatorkS = 0.0; // volts (V)
    static final double kElevatorkG = 0.762; // volts (V)
    static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
    static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    static final double kElevatorGearing = 10.0;
    static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    static final double kCarriageMass = 4.0; // kg

    static final double kSetpointMeters = 0.75;
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    static final double kMinElevatorHeightMeters = 0.0;
    static final double kMaxElevatorHeightMeters = 1.25;

    // distance per pulse = (distance per revolution) / (pulses per revolution)
    // = (Pi * D) / ppr
    static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 4096;

    // This gearbox represents a gearbox containing 4 Vex 775pro motors.
    private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);

    // Standard classes for controlling our elevator
    private final ProfiledPIDController m_controller = new ProfiledPIDController(
            kElevatorKp,
            kElevatorKi,
            kElevatorKd,
            new TrapezoidProfile.Constraints(2.45, 2.45));
    ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
            kElevatorkS,
            kElevatorkG,
            kElevatorkV,
            kElevatorkA);
    private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
    private final PWMSparkMax m_motor = new PWMSparkMax(kMotorPort);

    // Simulation classes help us simulate what's going on, including gravity.
    private final ElevatorSim m_elevatorSim = new ElevatorSim(
            m_elevatorGearbox,
            kElevatorGearing,
            kCarriageMass,
            kElevatorDrumRadius,
            kMinElevatorHeightMeters,
            kMaxElevatorHeightMeters,
            true,
            0,
            VecBuilder.fill(0.01));
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    private final PWMSim m_motorSim = new PWMSim(m_motor);

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d m_mech2d = new Mechanism2d(2, 2);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 1, 0);
    private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90, 30, new Color8Bit(235, 137, 52)));

    /** Subsystem constructor. */
    public Elevator() {
        m_encoder.setDistancePerPulse(kElevatorEncoderDistPerPulse);

        // Publish Mechanism2d to SmartDashboard
        // To view the Elevator visualization, select Network Tables -> SmartDashboard
        // -> Elevator Sim
        SmartDashboard.putData("Elevator Sim", m_mech2d);
    }

    @Override
    public void periodic() {
        // Update the telemetry, including mechanism visualization, regardless of mode.
        updateTelemetry();
        SmartDashboard.putNumber("elevator/goal", m_controller.getGoal().position);
        SmartDashboard.putNumber("elevator/position", m_encoder.getDistance());

        // With the setpoint value we run PID control like normal
        double pidOutput = m_controller.calculate(m_encoder.getDistance());
        double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
        m_motor.setVoltage(pidOutput + feedforwardOutput);
    }

    /** Advance the simulation. */
    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */
    public void setGoal(double goal) {
        m_controller.setGoal(goal);
    }

    public double getHeight() {
        return m_encoder.getDistance();
    }

    /** Stop the control loop and motor output. */
    public void stop() {
        m_controller.setGoal(0.0);
        m_motor.set(0.0);
    }

    /** Update telemetry, including the mechanism visualization. */
    public void updateTelemetry() {
        // Update elevator visualization with position
        m_elevatorMech2d.setLength(m_encoder.getDistance());
    }

    @Override
    public void close() {
        m_encoder.close();
        m_motor.close();
        m_mech2d.close();
    }
}