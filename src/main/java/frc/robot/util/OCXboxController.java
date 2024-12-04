package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Custom {@link CommandXboxController} wrapper to add convenience features for
 * driving.
 */
public class OCXboxController extends CommandXboxController {

    private static final double kDeadband = 0.12;

    public static final double kSpeedSlow = .3;
    public static final double kSpeedDefault = .75;
    public static final double kSpeedFast = 0.8;
    public static final double kSpeedMax = 1.0;
    
    private double drivespeed = kSpeedDefault;
    private double turnSpeed = kTurnSpeed;
    public static final double kTurnSpeedSlow = 0.15;
    public static final double kTurnSpeed = 0.35;

    /**
     * Constructs XboxController on DS joystick port.
     */
    public OCXboxController(int port) {
        super(port);
    }

    /**
     * Deadbands a value, re-scales it, and applies a power.
     * 
     * @param value Value to adjust
     * @return -1 to 1
     */
    public static double scaledPowerDeadband(double value, double exp) {
        value = MathUtil.applyDeadband(value, kDeadband);
        return Math.signum(value) * Math.pow(Math.abs(value), exp);
    }

    public void setDriveSpeed(double drivespeed) {
        this.drivespeed = drivespeed;
    }

    public double getDriveSpeed() {
        return drivespeed;
    }

    public void setTurnSpeed(double turnSpeed) {
        this.turnSpeed = turnSpeed;
    }

    public double getTurnSpeed() {
        return turnSpeed;
    }

    @Override
    public double getLeftY() {
        return getLeftY(1);
    }
    public double getLeftY(double exponent) {
        return -scaledPowerDeadband(super.getLeftY(), exponent);
    }
    @Override
    public double getLeftX() {
        return getLeftX(1);
    }
    public double getLeftX(double exponent) {
        return -scaledPowerDeadband(super.getLeftX(), exponent);
    }
    @Override
    public double getRightY() {
        return getRightY(1);
    }
    public double getRightY(double exponent) {
        return -scaledPowerDeadband(super.getRightY(), exponent);
    }
    @Override
    public double getRightX() {
        return getRightX(1);
    }
    public double getRightX(double exponent) {
        return -scaledPowerDeadband(super.getRightX(), exponent);
    }

    /**
     * Squares and applies deadband to left Y to give 'forward' percentage.
     * Affected by controller driveSpeed.
     * 
     * @return Percentage(-1 to 1)
     */
    public double getForward() {
        return getLeftY(2) * drivespeed;
    }

    /**
     * Squares and applies deadband to left X to give 'strafe' percentage.
     * Affected by controller driveSpeed.
     * 
     * @return Percentage(-1 to 1)
     */
    public double getStrafe() {
        return getLeftX(2) * drivespeed;
    }

    /**
     * Squares and applies deadband to right X to give 'turn' percentage.
     * Affected by controller turnSpeed.
     * 
     * @return Percentage(-1 to 1)
     */
    public double getTurn() {
        return getRightX(2) * turnSpeed;
    }

    /** Rumble both controller sides */
    public void rumble(double value){
        getHID().setRumble(RumbleType.kRightRumble, value);
        getHID().setRumble(RumbleType.kLeftRumble, value);
    }
    /** Rumble one controller side */
    public void rumble(boolean left, double value){
        RumbleType side = left ? RumbleType.kLeftRumble : RumbleType.kRightRumble;
        getHID().setRumble(side, value);
    }
}
