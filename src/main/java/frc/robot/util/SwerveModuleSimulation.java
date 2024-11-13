package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;
import org.dyn4j.geometry.Vector2;

/**
 *
 *
 * <h2>Simulation for a Single Swerve Module.</h2>
 *
 * <p>This class provides a simulation for a single swerve module in the {@link SwerveDriveSimulation}.
 *
 * <h3>1. Purpose</h3>
 *
 * <p>This class serves as the bridge between your code and the physics engine.
 *
 * <p>You will apply voltage outputs to the drive/steer motor of the module and obtain their encoder readings in your
 * code, just as how you deal with your physical motors.
 *
 * <h3>2. Perspectives</h3>
 *
 * <ul>
 *   <li>Simulates the steering mechanism using a custom brushless motor simulator.
 *   <li>Simulates the propelling force generated by the driving motor, with a current limit.
 *   <li>Simulates encoder readings, which can be used to simulate a {@link SwerveDriveOdometry}.
 * </ul>
 *
 * <h3>3. Simulating Odometry</h3>
 *
 * <ul>
 *   <li>Retrieve the encoder readings from {@link #getDriveEncoderUnGearedPositionRad()}} and
 *       {@link #getSteerAbsoluteFacing()}.
 *   <li>Use {@link SwerveDriveOdometry} to estimate the pose of your robot.
 *   <li><a
 *       href="https://v6.docs.ctr-electronics.com/en/latest/docs/application-notes/update-frequency-impact.html">250Hz
 *       Odometry</a> is supported. You can retrive cached encoder readings from every sub-tick through
 *       {@link #getCachedDriveEncoderUnGearedPositionsRad()} and {@link #getCachedSteerAbsolutePositions()}.
 * </ul>
 *
 * <p>An example of how to simulate odometry using this class is the <a
 * href='https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/AdvantageKit_AdvancedSwerveDriveProject/src/main/java/frc/robot/subsystems/drive/ModuleIOSim.java'>ModuleIOSim.java</a>
 * from the <code>Advanced Swerve Drive with maple-sim</code> example.
 */
public class SwerveModuleSimulation {
    public final DCMotor DRIVE_MOTOR;
    private final MapleMotorSim steerMotorSim;
    public final double DRIVE_CURRENT_LIMIT,
            DRIVE_GEAR_RATIO,
            STEER_GEAR_RATIO,
            DRIVE_FRICTION_VOLTAGE,
            WHEELS_COEFFICIENT_OF_FRICTION,
            WHEEL_RADIUS_METERS,
            DRIVE_WHEEL_INERTIA = 0.01;
    private double driveMotorRequestedVolts = 0.0,
            driveMotorAppliedVolts = 0.0,
            driveMotorSupplyCurrentAmps = 0.0,
            steerRelativeEncoderPositionRad = 0.0,
            steerRelativeEncoderSpeedRadPerSec = 0.0,
            steerAbsoluteEncoderSpeedRadPerSec = 0.0,
            driveEncoderUnGearedPositionRad = 0.0,
            driveEncoderUnGearedSpeedRadPerSec = 0.0;
    private Rotation2d steerAbsoluteFacing = Rotation2d.fromRotations(0);

    private final double steerRelativeEncoderOffSet = 0;

    private final Queue<Double> cachedSteerRelativeEncoderPositionsRad, cachedDriveEncoderUnGearedPositionsRad;
    private final Queue<Rotation2d> cachedSteerAbsolutePositions;

    /**
     *
     *
     * <h2>Constructs a Swerve Module Simulation.</h2>
     *
     * <p>If you are using {@link SimulatedArena#overrideSimulationTimings(double, int)} to use custom timings, you must
     * call the method before constructing any swerve module simulations using this constructor.
     *
     * @param driveMotor the model of the driving motor
     * @param steerMotor the model of the steering motor
     * @param driveCurrentLimit the current limit for the driving motor, in amperes
     * @param driveGearRatio the gear ratio for the driving motor, >1 is reduction
     * @param steerGearRatio the gear ratio for the steering motor, >1 is reduction
     * @param driveFrictionVoltage the measured minimum amount of voltage that can turn the driving rotter, in volts
     * @param steerFrictionVoltage the measured minimum amount of voltage that can turn the steering rotter, in volts
     * @param tireCoefficientOfFriction the <a
     *     href='https://simple.wikipedia.org/wiki/Coefficient_of_friction#:~:text=A%20coefficient%20of%20friction%20is%20a%20value%20that%20shows%20the'>coefficient
     *     of friction</a> of the tires, normally around 1.5
     * @param wheelsRadiusMeters the radius of the wheels, in meters. Calculate it using
     *     {@link Units#inchesToMeters(double)}.
     * @param steerRotationalInertia the rotational inertia of the steering mechanism
     */
    public SwerveModuleSimulation(
            DCMotor driveMotor,
            DCMotor steerMotor,
            double driveCurrentLimit,
            double driveGearRatio,
            double steerGearRatio,
            double driveFrictionVoltage,
            double steerFrictionVoltage,
            double tireCoefficientOfFriction,
            double wheelsRadiusMeters,
            double steerRotationalInertia) {
        DRIVE_MOTOR = driveMotor;
        DRIVE_CURRENT_LIMIT = driveCurrentLimit;
        DRIVE_GEAR_RATIO = driveGearRatio;
        STEER_GEAR_RATIO = steerGearRatio;
        DRIVE_FRICTION_VOLTAGE = driveFrictionVoltage;
        WHEELS_COEFFICIENT_OF_FRICTION = tireCoefficientOfFriction;
        WHEEL_RADIUS_METERS = wheelsRadiusMeters;

        this.steerMotorSim = new MapleMotorSim(
                SimulatedArena.getInstance(),
                steerMotor,
                steerGearRatio,
                KilogramSquareMeters.of(steerRotationalInertia),
                Volts.of(steerFrictionVoltage));

        this.cachedDriveEncoderUnGearedPositionsRad = new ConcurrentLinkedQueue<>();
        for (int i = 0; i < SimulatedArena.getSimulationSubTicksIn1Period(); i++)
            cachedDriveEncoderUnGearedPositionsRad.offer(driveEncoderUnGearedPositionRad);
        this.cachedSteerRelativeEncoderPositionsRad = new ConcurrentLinkedQueue<>();
        for (int i = 0; i < SimulatedArena.getSimulationSubTicksIn1Period(); i++)
            cachedSteerRelativeEncoderPositionsRad.offer(steerRelativeEncoderPositionRad);
        this.cachedSteerAbsolutePositions = new ConcurrentLinkedQueue<>();
        for (int i = 0; i < SimulatedArena.getSimulationSubTicksIn1Period(); i++)
            cachedSteerAbsolutePositions.offer(steerAbsoluteFacing);

        this.steerRelativeEncoderPositionRad = steerAbsoluteFacing.getRadians() + steerRelativeEncoderOffSet;
    }

    /**
     *
     *
     * <h2>Requests the Driving Motor to Run at a Specified Voltage Output.</h2>
     *
     * <h3>Think of it as the setVoltage() of your physical driving motor.</h3>
     *
     * <p>This method sets the desired voltage output for the driving motor. The change will be applied in the next
     * sub-tick of the simulation.
     *
     * <p><strong>Note:</strong> The requested voltage may not always be fully applied if the current is too high. The
     * current limit may reduce the motor's output, similar to real motors.
     *
     * <p>To check the actual voltage applied to the drivetrain, use {@link #getDriveMotorAppliedVolts()}.
     *
     * @param volts the voltage to be applied to the driving motor
     */
    public void requestDriveVoltageOut(double volts) {
        this.driveMotorRequestedVolts = volts;
    }

    /**
     *
     *
     * <h2>Requests the Steering Motor to Run at a Specified Voltage Output.</h2>
     *
     * <h3>Think of it as the setVoltage() of your physical steering motor.</h3>
     *
     * <p>This method sets the desired voltage output for the steering motor. The change will be applied in the next
     * sub-tick of the simulation.
     *
     * <p><strong>Note:</strong> Similar to the drive motor, the requested voltage may not always be fully applied if
     * the current exceeds the limit. The current limit will reduce the motor's output as needed, mimicking real motor
     * behavior.
     *
     * <p>To check the actual voltage applied to the steering motor, use {@link #getSteerMotorAppliedVolts()}.
     *
     * @param volts the voltage to be applied to the steering motor
     */
    public void requestSteerVoltageOut(double volts) {
        this.steerMotorSim.setControl(Volts.of(volts));
    }

    /**
     *
     *
     * <h2>Obtains the Actual Output Voltage of the Drive Motor.</h2>
     *
     * <p>This method returns the actual voltage being applied to the drive motor. The actual applied voltage may differ
     * from the value set by {@link #requestDriveVoltageOut(double)}.
     *
     * <p>If the motor's supply current is too high, the motor will automatically reduce its output voltage to protect
     * the system.
     *
     * @return the actual output voltage of the drive motor, in volts
     */
    public double getDriveMotorAppliedVolts() {
        return driveMotorAppliedVolts;
    }

    /**
     *
     *
     * <h2>Obtains the Actual Output Voltage of the Steering Motor.</h2>
     *
     * <p>This method returns the actual voltage being applied to the steering motor. It wraps around the
     * {@link MapleMotorSim#getAppliedVolts()} method.
     *
     * <p>The actual applied voltage may differ from the value set by {@link #requestSteerVoltageOut(double)}. If the
     * motor's supply current is too high, the motor will automatically reduce its output voltage to protect the system.
     *
     * @return the actual output voltage of the steering motor, in volts
     */
    public double getSteerMotorAppliedVolts() {
        return steerMotorSim.getRotorVoltage().in(Volts);
    }

    /**
     *
     *
     * <h2>Obtains the Amount of Current Supplied to the Drive Motor.</h2>
     *
     * <h3>Think of it as the getSupplyCurrent() of your physical drive motor.</h3>
     *
     * @return the current supplied to the drive motor, in amperes
     */
    public double getDriveMotorSupplyCurrentAmps() {
        return driveMotorSupplyCurrentAmps;
    }

    /**
     *
     *
     * <h2>Obtains the Amount of Current Supplied to the Steer Motor.</h2>
     *
     * <h3>Think of it as the getSupplyCurrent() of your physical steer motor.</h3>
     *
     * <p>This method wraps around {@link MapleMotorSim#getCurrentDrawAmps()}.
     *
     * @return the current supplied to the steer motor, in amperes
     */
    public double getSteerMotorSupplyCurrentAmps() {
        return steerMotorSim.getSupplyCurrent().in(Amps);
    }

    /**
     *
     *
     * <h2>Obtains the Position of the Drive Encoder.</h2>
     *
     * <h3>Think of it as the getPosition() of your physical driving motor.</h3>
     *
     * <p>This method is used to simulate your {@link SwerveDrivePoseEstimator}.
     *
     * <p>This value represents the un-geared position of the encoder, i.e., the amount of radians the drive motor's
     * encoder has rotated.
     *
     * <p>To get the final wheel rotations, use {@link #getDriveWheelFinalPositionRad()}.
     *
     * @return the position of the drive motor's encoder, in radians (un-geared)
     */
    public double getDriveEncoderUnGearedPositionRad() {
        return driveEncoderUnGearedPositionRad;
    }

    /**
     *
     *
     * <h2>Obtains the Final Position of the Drive Encoder (Wheel Rotations).</h2>
     *
     * <p>This method is used to simulate the {@link SwerveDrivePoseEstimator} by providing the final position of the
     * drive encoder in terms of wheel rotations.
     *
     * <p>The value is calculated by dividing the un-geared encoder position (obtained from
     * {@link #getDriveEncoderUnGearedPositionRad()}) by the drive gear ratio.
     *
     * @return the final position of the drive encoder (wheel rotations), in radians
     */
    public double getDriveWheelFinalPositionRad() {
        return getDriveEncoderUnGearedPositionRad() / DRIVE_GEAR_RATIO;
    }

    /**
     *
     *
     * <h2>Obtains the Speed of the Drive Encoder (Un-Geared), in Radians per Second.</h2>
     *
     * <h3>Think of it as the <code>getVelocity()</code> of your physical drive motor.</h3>
     *
     * <p>This method returns the current speed of the drive encoder in radians per second, without accounting for the
     * drive gear ratio.
     *
     * @return the un-geared speed of the drive encoder, in radians per second
     */
    public double getDriveEncoderUnGearedSpeedRadPerSec() {
        return driveEncoderUnGearedSpeedRadPerSec;
    }

    /**
     *
     *
     * <h2>Obtains the Final Speed of the Wheel, in Radians per Second.</h2>
     *
     * @return the final speed of the drive wheel, in radians per second
     */
    public double getDriveWheelFinalSpeedRadPerSec() {
        return getDriveEncoderUnGearedSpeedRadPerSec() / DRIVE_GEAR_RATIO;
    }

    /**
     *
     *
     * <h2>Obtains the Relative Position of the Steer Encoder, in Radians.</h2>
     *
     * <h3>Think of it as the <code>getPosition()</code> of your physical steer motor.</h3>
     *
     * @return the relative encoder position of the steer motor, in radians
     */
    public double getSteerRelativeEncoderPositionRad() {
        return steerRelativeEncoderPositionRad;
    }

    /**
     *
     *
     * <h2>Obtains the Speed of the Steer Relative Encoder, in Radians per Second (Geared).</h2>
     *
     * <h3>Think of it as the <code>getVelocity()</code> of your physical steer motor.</h3>
     *
     * @return the speed of the steer relative encoder, in radians per second (geared)
     */
    public double getSteerRelativeEncoderSpeedRadPerSec() {
        return steerRelativeEncoderSpeedRadPerSec;
    }

    /**
     *
     *
     * <h2>Obtains the Absolute Facing of the Steer Mechanism.</h2>
     *
     * <h3>Think of it as the <code>getAbsolutePosition()</code> of your CanCoder.</h3>
     *
     * @return the absolute facing of the steer mechanism, as a {@link Rotation2d}
     */
    public Rotation2d getSteerAbsoluteFacing() {
        return steerAbsoluteFacing;
    }

    /**
     *
     *
     * <h2>Obtains the Absolute Rotational Velocity of the Steer Mechanism.</h2>
     *
     * <h3>Think of it as the <code>getVelocity()</code> of your CanCoder.</h3>
     *
     * @return the absolute angular velocity of the steer mechanism, in radians per second
     */
    public double getSteerAbsoluteEncoderSpeedRadPerSec() {
        return steerAbsoluteEncoderSpeedRadPerSec;
    }

    /**
     *
     *
     * <h2>Obtains the Cached Readings of the Drive Encoder's Un-Geared Position.</h2>
     *
     * <p>The values of {@link #getDriveEncoderUnGearedPositionRad()} are cached at each sub-tick and can be retrieved
     * using this method.
     *
     * @return an array of cached drive encoder un-geared positions, in radians
     */
    public double[] getCachedDriveEncoderUnGearedPositionsRad() {
        return cachedDriveEncoderUnGearedPositionsRad.stream()
                .mapToDouble(value -> value)
                .toArray();
    }

    /**
     *
     *
     * <h2>Obtains the Cached Readings of the Drive Encoder's Final Position (Wheel Rotations).</h2>
     *
     * <p>The values of {@link #getDriveEncoderUnGearedPositionRad()} are cached at each sub-tick and are divided by the
     * gear ratio to obtain the final wheel rotations.
     *
     * @return an array of cached drive encoder final positions (wheel rotations), in radians
     */
    public double[] getCachedDriveWheelFinalPositionsRad() {
        return cachedDriveEncoderUnGearedPositionsRad.stream()
                .mapToDouble(value -> value / DRIVE_GEAR_RATIO)
                .toArray();
    }

    /**
     *
     *
     * <h2>Obtains the Cached Readings of the Steer Relative Encoder's Position.</h2>
     *
     * <p>The values of {@link #getSteerRelativeEncoderPositionRad()} are cached at each sub-tick and can be retrieved
     * using this method.
     *
     * @return an array of cached steer relative encoder positions, in radians
     */
    public double[] getCachedSteerRelativeEncoderPositions() {
        return cachedSteerRelativeEncoderPositionsRad.stream()
                .mapToDouble(value -> value)
                .toArray();
    }

    /**
     *
     *
     * <h2>Obtains the Cached Readings of the Steer Absolute Positions.</h2>
     *
     * <p>The values of {@link #getSteerAbsoluteFacing()} are cached at each sub-tick and can be retrieved using this
     * method.
     *
     * @return an array of cached absolute steer positions, as {@link Rotation2d} objects
     */
    public Rotation2d[] getCachedSteerAbsolutePositions() {
        return cachedSteerAbsolutePositions.toArray(Rotation2d[]::new);
    }

    protected double getGrippingForceNewtons(double gravityForceOnModuleNewtons) {
        return gravityForceOnModuleNewtons * WHEELS_COEFFICIENT_OF_FRICTION;
    }

    /**
     *
     *
     * <h2>Updates the Simulation for This Module.</h2>
     *
     * <p>This method is called once during every sub-tick of the simulation. It performs the following actions:
     *
     * <ul>
     *   <li>Updates the simulation of the steering mechanism.
     *   <li>Simulates the propelling force generated by the module.
     *   <li>Updates and caches the encoder readings for odometry simulation.
     * </ul>
     *
     * <p><strong>Note:</strong> Friction forces are not simulated in this method.
     *
     * @param moduleCurrentGroundVelocityWorldRelative the current ground velocity of the module, relative to the world
     * @param robotFacing the absolute facing of the robot, relative to the world
     * @param gravityForceOnModuleNewtons the gravitational force acting on this module, in newtons
     * @return the propelling force generated by the module, as a {@link Vector2} object
     */
    public Vector2 updateSimulationSubTickGetModuleForce(
            Vector2 moduleCurrentGroundVelocityWorldRelative,
            Rotation2d robotFacing,
            double gravityForceOnModuleNewtons) {
        updateSteerSimulation();

        /* the maximum gripping force that the wheel can generate */
        final double grippingForceNewtons = getGrippingForceNewtons(gravityForceOnModuleNewtons);
        final Rotation2d moduleWorldFacing = this.steerAbsoluteFacing.plus(robotFacing);
        final Vector2 propellingForce =
                getPropellingForce(grippingForceNewtons, moduleWorldFacing, moduleCurrentGroundVelocityWorldRelative);
        updateDriveEncoders();

        return propellingForce;
    }

    /**
     *
     *
     * <h2>updates the simulation for the steer mechanism.</h2>
     *
     * <p>Updates the simulation for the steer mechanism and cache the encoder readings.
     *
     * <p>The steer mechanism is modeled by a {@link MapleMotorSim}.
     */
    private void updateSteerSimulation() {
        /* update the readings of the sensor */
        this.steerAbsoluteFacing = new Rotation2d(steerMotorSim.getPosition());
        this.steerRelativeEncoderPositionRad = steerMotorSim.getPosition().in(Radians) + steerRelativeEncoderOffSet;
        this.steerAbsoluteEncoderSpeedRadPerSec = steerMotorSim.getVelocity().in(RadiansPerSecond);
        this.steerRelativeEncoderSpeedRadPerSec = steerAbsoluteEncoderSpeedRadPerSec * STEER_GEAR_RATIO;

        /* cache sensor readings to queue for high-frequency odometry */
        this.cachedSteerAbsolutePositions.poll();
        this.cachedSteerAbsolutePositions.offer(steerAbsoluteFacing);
        this.cachedSteerRelativeEncoderPositionsRad.poll();
        this.cachedSteerRelativeEncoderPositionsRad.offer(steerRelativeEncoderPositionRad);
    }

    /**
     *
     *
     * <h2>Calculates the amount of propelling force that the module generates.</h2>
     *
     * <p>The swerve module's drive motor will generate a propelling force.
     *
     * <p>For most of the time, that propelling force is directly applied to the drivetrain. And the drive wheel runs as
     * fast as the ground velocity
     *
     * <p>However, if the propelling force exceeds the gripping, only the max gripping force is applied. The rest of the
     * propelling force will cause the wheel to start skidding and make the odometry inaccurate.
     *
     * @param grippingForceNewtons the amount of gripping force that wheel can generate, in newtons
     * @param moduleWorldFacing the current world facing of the module
     * @param moduleCurrentGroundVelocity the current ground velocity of the module, world-reference
     * @return a vector representing the propelling force that the module generates, world-reference
     */
    private Vector2 getPropellingForce(
            double grippingForceNewtons, Rotation2d moduleWorldFacing, Vector2 moduleCurrentGroundVelocity) {
        final double driveWheelTorque = getDriveWheelTorque(),
                theoreticalMaxPropellingForceNewtons = driveWheelTorque / WHEEL_RADIUS_METERS;
        final boolean skidding = Math.abs(theoreticalMaxPropellingForceNewtons) > grippingForceNewtons;
        final double propellingForceNewtons;
        if (skidding)
            propellingForceNewtons = Math.copySign(grippingForceNewtons, theoreticalMaxPropellingForceNewtons);
        else propellingForceNewtons = theoreticalMaxPropellingForceNewtons;

        final double floorVelocityProjectionOnWheelDirectionMPS = moduleCurrentGroundVelocity.getMagnitude()
                * Math.cos(moduleCurrentGroundVelocity.getAngleBetween(new Vector2(moduleWorldFacing.getRadians())));

        // if the chassis is tightly gripped on floor, the floor velocity is projected to the wheel
        this.driveEncoderUnGearedSpeedRadPerSec =
                floorVelocityProjectionOnWheelDirectionMPS / WHEEL_RADIUS_METERS * DRIVE_GEAR_RATIO;

        // if the module is skidding
        if (skidding)
            this.driveEncoderUnGearedSpeedRadPerSec =
                    0.5 * driveEncoderUnGearedSpeedRadPerSec + 0.5 * DRIVE_MOTOR.getSpeed(0, driveMotorAppliedVolts);

        return Vector2.create(propellingForceNewtons, moduleWorldFacing.getRadians());
    }

    /**
     *
     *
     * <h2>Calculates the amount of torque that the drive motor can generate on the wheel.</h2>
     *
     * <p>Before calculating the torque of the motor, the output voltage of the drive motor is constrained for the
     * current limit through {@link MapleMotorSim#constrainOutputVoltage(DCMotor, double, double, double)}.
     *
     * @return the amount of torque on the wheel by the drive motor, in Newton * Meters
     */
    private double getDriveWheelTorque() {
        driveMotorAppliedVolts = driveMotorRequestedVolts;

        /* calculate the actual supply current */
        driveMotorSupplyCurrentAmps = DRIVE_MOTOR.getCurrent(
                this.driveEncoderUnGearedSpeedRadPerSec,
                MathUtil.applyDeadband(driveMotorAppliedVolts, DRIVE_FRICTION_VOLTAGE, 12));

        /* calculate the torque generated,  */
        final double torqueOnRotter = DRIVE_MOTOR.getTorque(driveMotorSupplyCurrentAmps);
        return torqueOnRotter * DRIVE_GEAR_RATIO;
    }

    /** @return the current module state of this simulation module */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveWheelFinalSpeedRadPerSec() * WHEEL_RADIUS_METERS, steerAbsoluteFacing);
    }

    /**
     *
     *
     * <h2>Obtains the "free spin" state of the module</h2>
     *
     * <p>The "free spin" state of a simulated module refers to its state after spinning freely for a long time under
     * the current input voltage
     *
     * @return the free spinning module state
     */
    protected SwerveModuleState getFreeSpinState() {
        return new SwerveModuleState(
                DRIVE_MOTOR.getSpeed(
                                DRIVE_MOTOR.getTorque(DRIVE_MOTOR.getCurrent(0, DRIVE_FRICTION_VOLTAGE)),
                                driveMotorAppliedVolts)
                        / DRIVE_GEAR_RATIO
                        * WHEEL_RADIUS_METERS,
                steerAbsoluteFacing);
    }

    /**
     *
     *
     * <h2>Cache the encoder values.</h2>
     *
     * <p>An internal method to cache the encoder values to their queues.
     */
    private void updateDriveEncoders() {
        this.driveEncoderUnGearedPositionRad +=
                this.driveEncoderUnGearedSpeedRadPerSec * SimulatedArena.getSimulationDt();
        this.cachedDriveEncoderUnGearedPositionsRad.poll();
        this.cachedDriveEncoderUnGearedPositionsRad.offer(driveEncoderUnGearedPositionRad);
    }

    /**
     *
     *
     * <h2>Obtains the theoretical speed that the module can achieve.</h2>
     *
     * @return the theoretical maximum ground speed that the module can achieve, in m/s
     */
    public double getModuleTheoreticalSpeedMPS() {
        return DRIVE_MOTOR.freeSpeedRadPerSec / DRIVE_GEAR_RATIO * WHEEL_RADIUS_METERS;
    }

    /**
     *
     *
     * <h2>Obtains the theoretical maximum propelling force of ONE module.</h2>
     *
     * <p>Calculates the maximum propelling force with respect to the gripping force and the drive motor's torque under
     * its current limit.
     *
     * @param robotMassKg the mass of the robot, is kilograms
     * @param modulesCount the amount of modules on the robot, assumed to be sharing the gravity force equally
     * @return the maximum propelling force of EACH module
     */
    public double getTheoreticalPropellingForcePerModule(double robotMassKg, int modulesCount) {
        final double
                maxThrustNewtons = DRIVE_MOTOR.getTorque(DRIVE_CURRENT_LIMIT) * DRIVE_GEAR_RATIO / WHEEL_RADIUS_METERS,
                maxGrippingNewtons = 9.8 * robotMassKg / modulesCount * WHEELS_COEFFICIENT_OF_FRICTION;

        return Math.min(maxThrustNewtons, maxGrippingNewtons);
    }

    /**
     *
     *
     * <h2>Obtains the theatrical linear acceleration that the robot can achieve.</h2>
     *
     * <p>Calculates the maximum linear acceleration of a robot, with respect to its mass and
     * {@link #getTheoreticalPropellingForcePerModule(double, int)}.
     *
     * @param robotMassKg the mass of the robot, is kilograms
     * @param modulesCount the amount of modules on the robot, assumed to be sharing the gravity force equally
     */
    public double getModuleMaxAccelerationMPSsq(double robotMassKg, int modulesCount) {
        return getTheoreticalPropellingForcePerModule(robotMassKg, modulesCount) * modulesCount / robotMassKg;
    }

    /**
     *
     *
     * <h2>Stores the coefficient of friction of some common used wheels.</h2>
     */
    public enum WHEEL_GRIP {
        RUBBER_WHEEL(1.25),
        TIRE_WHEEL(1.2);

        public final double cof;

        WHEEL_GRIP(double cof) {
            this.cof = cof;
        }
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module">SDS Mark4
     * Swerve Module</a> for simulation
     */
    public static Supplier<SwerveModuleSimulation> getMark4(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 8.14;
                    case 2 -> 6.75;
                    case 3 -> 6.12;
                    case 4 -> 5.14;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                12.8,
                0.2,
                0.3,
                wheelCOF,
                Units.inchesToMeters(2),
                0.03);
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module">SDS
     * Mark4-i Swerve Module</a> for simulation
     */
    public static Supplier<SwerveModuleSimulation> getMark4i(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 8.14;
                    case 2 -> 6.75;
                    case 3 -> 6.12;
                    case 4 -> 5.15;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                150.0 / 7.0,
                0.2,
                1,
                wheelCOF,
                Units.inchesToMeters(2),
                0.025);
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/products/mk4n-swerve-module">SDS Mark4-n Swerve
     * Module</a> for simulation
     */
    public static Supplier<SwerveModuleSimulation> getMark4n(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 7.13;
                    case 2 -> 5.9;
                    case 3 -> 5.36;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                18.75,
                0.25,
                1,
                wheelCOF,
                Units.inchesToMeters(2),
                0.025);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x">WCP SwerveX Swerve Module</a>
     * for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    public static Supplier<SwerveModuleSimulation> getSwerveX(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 7.85;
                    case 2 -> 7.13;
                    case 3 -> 6.54;
                    case 4 -> 6.56;
                    case 5 -> 5.96;
                    case 6 -> 5.46;
                    case 7 -> 5.14;
                    case 8 -> 4.75;
                    case 9 -> 4.41;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                11.3142,
                0.2,
                0.3,
                wheelCOF,
                Units.inchesToMeters(2),
                0.03);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x-flipped">WCP SwerveX Flipped
     * Swerve Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    public static Supplier<SwerveModuleSimulation> getSwerveXFlipped(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 8.1;
                    case 2 -> 7.36;
                    case 3 -> 6.75;
                    case 4 -> 6.72;
                    case 5 -> 6.11;
                    case 6 -> 5.6;
                    case 7 -> 5.51;
                    case 8 -> 5.01;
                    case 9 -> 4.59;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                11.3714,
                0.2,
                0.3,
                wheelCOF,
                Units.inchesToMeters(2),
                0.03);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-xs">WCP SwerveXS Swerve
     * Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6
     */
    public static Supplier<SwerveModuleSimulation> getSwerveXS(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 6.0;
                    case 2 -> 5.54;
                    case 3 -> 5.14;
                    case 4 -> 4.71;
                    case 5 -> 4.4;
                    case 6 -> 4.13;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                41.25,
                0.2,
                0.3,
                wheelCOF,
                Units.inchesToMeters(1.5),
                0.03);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x2">WCP SwerveX2 Swerve
     * Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9 <br>
     * X4 Ratios are gearRatioLevel 10-12
     */
    public static Supplier<SwerveModuleSimulation> getSwerveX2(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 7.67;
                    case 2 -> 6.98;
                    case 3 -> 6.39;
                    case 4 -> 6.82;
                    case 5 -> 6.20;
                    case 6 -> 5.68;
                    case 7 -> 6.48;
                    case 8 -> 5.89;
                    case 9 -> 5.40;
                    case 10 -> 5.67;
                    case 11 -> 5.15;
                    case 12 -> 4.73;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                12.1,
                0.2,
                0.3,
                wheelCOF,
                Units.inchesToMeters(2),
                0.03);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x2-s">WCP SwerveX2S Swerve
     * Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    public static Supplier<SwerveModuleSimulation> getSwerveX2S(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 6.0;
                    case 2 -> 5.63;
                    case 3 -> 5.29;
                    case 4 -> 4.94;
                    case 5 -> 4.67;
                    case 6 -> 4.42;
                    case 7 -> 4.11;
                    case 8 -> 3.9;
                    case 9 -> 3.71;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                25.9,
                0.2,
                0.3,
                wheelCOF,
                Units.inchesToMeters(1.875),
                0.03);
    }
}
