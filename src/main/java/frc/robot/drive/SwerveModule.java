package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.control.motors.NKTalonFX;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private NKTalonFX drive;
    private NKTalonFX turn;
    private CANCoder turnEncoder;
    private Rotation2d angleOffset;
    private SimpleMotorFeedforward feedforward;

    private int id;

    private double lastAngle;

    public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, Rotation2d angleOffset) {
        this.angleOffset = angleOffset;
        id = (driveMotorID / 10);
        initEncoder(encoderID);
        initDriveMotor(driveMotorID);
        initTurnMotor(turnMotorID);
        this.feedforward = new SimpleMotorFeedforward(ModuleConstants.kDriveS, ModuleConstants.kDriveV,
                ModuleConstants.kDriveA);
    }

    public SwerveModuleState getCurrentState() {
        double velocity = getVelocityMPS();
        Rotation2d angle = getAngleRotation2d();
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double distance = getDistanceMeters();
        Rotation2d angle = getAngleRotation2d();
        return new SwerveModulePosition(distance, angle);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        // if falcon resets, set the integrated encoder back to absolute position
        if (turn.hasResetOccurred()) {
            resetToAbsolute();
        }

        // optimize angle so that the module never turns more than 90 degrees
        Rotation2d currentAngleRotation2d = getAngleRotation2d();
        desiredState = SwerveModuleState.optimize(desiredState, currentAngleRotation2d);

        // set angle and velocity
        setAngle(desiredState, currentAngleRotation2d);
        setVelocity(desiredState, isOpenLoop);
    }

    private void setAngle(SwerveModuleState desiredState, Rotation2d currentAngleRotation2d) {

        // calculate angle for the falcon to go to
        double angle = 0.0;
        if (Math.abs(desiredState.speedMetersPerSecond) <= (ModuleConstants.kMaxSpeed * 0.01)) {
            // if moving, use the desired state's angle
            angle = desiredState.angle.getDegrees();
        } else {
            // if not moving, use the last given angle
            angle = lastAngle;
        }

        // sets the turn motor
        if (Math.abs(currentAngleRotation2d.minus(desiredState.angle)
                .getDegrees()) > ModuleConstants.kAllowableAngleTolerance.getDegrees()) {
            // if angle is bigger than tolerance, set angle
            turn.set(ControlMode.Position, Conversions.degreesToFalcon(angle, ModuleConstants.kTurnGearRatio));
        } else {
            // if angle is smaller than tolerance, set to zero
            turn.set(ControlMode.PercentOutput, 0);
        }

        // update last angle
        lastAngle = angle;
    }

    private void setVelocity(SwerveModuleState desiredState, boolean isOpenLoop) {
        // set the drive velocity
        if (isOpenLoop) {
            // estimates percentage of motor and sets it
            double percentOutput = desiredState.speedMetersPerSecond / ModuleConstants.kMaxSpeed;
            drive.set(ControlMode.PercentOutput, percentOutput);
        } else {
            // convert desired speed in m/s to falcon units
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    ModuleConstants.kWheelCircumference, ModuleConstants.kDriveGearRatio);

            // set velocity using feedforward
            drive.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void resetToAbsolute() {
        lastAngle = getCANCoder().minus(angleOffset).getDegrees();
        double absolutePosition = -Conversions.degreesToFalcon(lastAngle, ModuleConstants.kTurnGearRatio);
        turn.setSelectedSensorPosition(absolutePosition);
    }

    private void initDriveMotor(int driveMotorID) {
        drive = new NKTalonFX(driveMotorID);

        drive.configFactoryDefault();
        drive.configAllSettings(ModuleConstants.kDriveMotorConfig);
        drive.setInverted(ModuleConstants.kDriveMotorInverted);
        drive.setNeutralMode(ModuleConstants.kDriveMotorNeutral);
        drive.setSelectedSensorPosition(0);

    }

    private void initTurnMotor(int turnMotorID) {
        turn = new NKTalonFX(turnMotorID);
        turn.configFactoryDefault();
        turn.configAllSettings(ModuleConstants.kTurnMotorConfig);
        turn.setInverted(ModuleConstants.kTurnMotorInverted);
        turn.setNeutralMode(ModuleConstants.kTurnMotorNeutral);
        turn.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        resetToAbsolute();
    }

    private void initEncoder(int encoderID) {
        turnEncoder = new CANCoder(encoderID);

        turnEncoder.configFactoryDefault();
        turnEncoder.configAllSettings(ModuleConstants.kEncoderConfig);
    }

    public void updateSmartDash() {
        // SmartDashboard.putNumber(id + " Module Encoder Raw Position",
        // turnEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber(id + " Motor Integrated Sensor Position",
        // turn.getSelectedSensorPosition());
        // SmartDashboard.putNumber(id + " Module Angle",
        // getAngleRotation2d().getDegrees());
        // SmartDashboard.putNumber(id + " cancoder - offset",
        // getCANCoder().minus(angleOffset).getDegrees());

        SmartDashboard.putNumber(id + " module velocity mps", this.getVelocityMPS());
    }

    private Rotation2d getCANCoder() {
        return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition());
    }

    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(turn.getSelectedSensorPosition(), ModuleConstants.kTurnGearRatio));
    }

    public double getDistanceMeters() {
        double distance = Conversions.falconToMeters(drive.getSelectedSensorPosition(),
                ModuleConstants.kWheelCircumference, ModuleConstants.kDriveGearRatio);
        return distance;
    }

    public double getVelocityMPS() {
        return Conversions.falconToMPS(drive.getSelectedSensorVelocity(), ModuleConstants.kWheelCircumference,
                ModuleConstants.kDriveGearRatio);
    }

    public void testDriveSpinny(double output) {
        drive.set(output);
    }

    public void testTurnSpinny(double output) {
        turn.set(output);
    }

    public void testStopSpinny() {
        drive.set(0);
        turn.set(0);
    }

    public void setDriveVoltage(double voltage) {
        drive.setVoltage(voltage);
    }

    public static final class Conversions {

        public static final double kSecondsPerMinute = 60.0;
        public static final double kFalconTicks = 2048.0;
        public static final double kMaxFalconRPM = 6380.0;
        public static final double kFalconToDegrees = 360.0 / kFalconTicks;
        public static final double kFalconToRotations = 1.0 / kFalconTicks;

        // starting units: ticks/100ms
        // ticks/100ms * (1000ms/s * 60s/m) * 1 rot/ 2048 ticks = 600 ticks/m * 1
        // rot/ticks
        public static final double kFalconVelocityToRPM = 600.0 / kFalconTicks;

        public static double falconToDegrees(double counts, double gearRatio) {
            // ratio = motor/wheel
            return counts * kFalconToDegrees / gearRatio;
        }

        public static double degreesToFalcon(double degrees, double gearRatio) {
            double ticks = degrees * gearRatio / kFalconToDegrees;
            return ticks;
        }

        /**
         * Converts a falcon motor position into distance traveled
         * 
         * @param falconPosition falcon position sensor counts
         * @param circumference  wheel circumference in meters
         * @param gearRatio      motor rotations/wheel rotations
         * @return distance traveled in meters
         */
        public static double falconToMeters(double falconPosition, double circumference, double gearRatio) {
            double wheelRotations = falconPosition * kFalconToRotations / gearRatio;
            double distance = wheelRotations * circumference;
            return distance;
        }

        public static double falconToRPM(double velocityCounts, double gearRatio) {
            double motorRPM = velocityCounts * kFalconVelocityToRPM;
            double mechRPM = motorRPM / gearRatio;
            return mechRPM;
        }

        public static double RPMToFalcon(double RPM, double gearRatio) {
            double motorRPM = RPM * gearRatio;
            double sensorCounts = motorRPM / kFalconVelocityToRPM;
            return sensorCounts;
        }

        public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
            double wheelRPM = falconToRPM(velocitycounts, gearRatio);
            double wheelMPS = (wheelRPM * circumference) / kSecondsPerMinute;
            return wheelMPS;
        }

        public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
            double wheelRPM = ((velocity * kSecondsPerMinute) / circumference);
            double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
            return wheelVelocity;
        }
    }
}
