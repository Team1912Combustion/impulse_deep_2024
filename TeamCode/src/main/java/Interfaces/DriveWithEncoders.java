package Interfaces;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import Interfaces.Drive;
import Interfaces.IGyro;
import Interfaces.EHimu;
import Interfaces.Encoder;

public class DriveWithEncoders implements Drive {
    private PIDController _PIDDriveDistance;
    private PIDController _PIDDriveStraight;
    private PIDController _PIDRotate;

    private DcMotor _WheelFrontLeft;
    private DcMotor _WheelFrontRight;
    private DcMotor _WheelBackLeft;
    private DcMotor _WheelBackRight;
    private DcMotor _Intake;
    private DcMotor _OdometerLeft;
    private DcMotor _OdometerRight;
    private double _power = 0.2;
    private double _targetDistance;
    private IGyro _EHGyro;

    private double _correction;
    private double _leftCurrentPosition = 0;
    private double _leftTargetPosition = 0;
    private double _rightCurrentPosition = 0;
    private double _rightTargetPosition = 0;

    private final long _MILLS_TO_SLEEP = 1000;

    private static final double  TICKS_PER_INCH = 336.88;
    private final double TICKS_PER_REVOLUTION = 2000;
    private final double DEGREES_PER_TICK = 360/TICKS_PER_REVOLUTION;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // Range Sensor
    RangeSensor rangeSensor;
    boolean stopDistanceReached = false;

    public DriveWithEncoders (HardwareMap hardwareMap, Telemetry telemetry){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        Initialize();

    }

    private void Initialize(){


        // Initialize Wheels
        _WheelFrontLeft = hardwareMap.dcMotor.get("WheelFL");
        _WheelFrontRight = hardwareMap.dcMotor.get("WheelFR");
        _WheelBackLeft = hardwareMap.dcMotor.get("WheelBL");
        _WheelBackRight = hardwareMap.dcMotor.get("WheelBR");
        _Intake = hardwareMap.dcMotor.get("Intake");


        _WheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _WheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _WheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _WheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _WheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _WheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _WheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _WheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        _WheelFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        _WheelFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        _WheelBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        _WheelBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        _Intake.setDirection(DcMotorSimple.Direction.FORWARD);

//        _WheelFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        _WheelFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        _WheelBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        _WheelBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        _WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Encoders
        _OdometerLeft = hardwareMap.dcMotor.get("WheelFR");
        _OdometerRight = hardwareMap.dcMotor.get("WheelFL");

        _OdometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _OdometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _OdometerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _OdometerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        _OdometerLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        _OdometerRight.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        _OdometerLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        _OdometerRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize PID

        /* Set PID proportional value to start reducing power at about 50 degrees of rotation.
         * P by itself may stall before turn completed so we add a bit of I (integral) which
         * causes the PID controller to gently increase power if the turn is not completed. */
        _PIDRotate = new PIDController(0, 0, 0);

        /* Set PID proportional value to produce non-zero correction value when robot veers off
         * straight line. P value controls how sensitive the correction is. */
        _PIDDriveDistance = new PIDController(0, 0, 0);
        _PIDDriveStraight = new PIDController(0.05, 0, 0);

        _EHGyro = new EHimu(hardwareMap, telemetry);

        // Initialize Range Sensor
        rangeSensor = new RangeSensor(hardwareMap, telemetry);

    }

    @Override
    public void Forward(double distanceInch, double power, double stopDistance) {
        sleep(100);
        stopDistanceReached = false;

        _targetDistance = -Math.abs(InchesToDegrees(distanceInch)); // Encoder values decrease driving forward, hence a negative target distance.
        _power = -Math.abs(power);                                  // Power will be negative driving forward.
        ResetEncoders();

        // Update current and target distances
        UpdateCurrentPositions();
        UpdateTargetPositions(_targetDistance);

        // Set up parameters for driving in a straight line.
        ResetPIDDriveDistance();
        ResetPIDDriveStraight();

        do { // Drive until we reach the target distance
            UpdateCurrentPositions();
            _power = _PIDDriveDistance.performPID(_leftCurrentPosition);
            _correction =0;// _PIDDriveStraight.performPID(_EHGyro.GetHeadingEH());


            ShowTelemetry();

            // TODO: We may need this.
//            if (_power + _correction >= -0.2) {
//                _power -= _correction;
//            }

            _WheelBackLeft.setPower(_power + _correction);
            _WheelBackRight.setPower(_power - _correction);
            _WheelFrontLeft.setPower(_power + _correction);
            _WheelFrontRight.setPower(_power - _correction);

            UpdateCurrentPositions();

            if (rangeSensor.GetDistance() <= stopDistance)
                stopDistanceReached = true;

        } while (!_PIDDriveDistance.onTarget() || !stopDistanceReached);

        StopRobot();

        ShowTelemetry();

        // reset angle tracking on new heading.
        _EHGyro.ResetHeadingEH();

        sleep(_MILLS_TO_SLEEP);
    }

    @Override
    public void Backward(double distanceInch, double power) {
        sleep(100);

        _targetDistance = Math.abs(InchesToDegrees(distanceInch)); // Encoder values Increase driving backward, hence a positive target distance.
        _power = Math.abs(power);                                  // Power will be positive driving backward.
        ResetEncoders();

        // Update current and target distances
        UpdateCurrentPositions();
        UpdateTargetPositions(_targetDistance);

        // Set up parameters for driving in a straight line.
        ResetPIDDriveDistance();
        ResetPIDDriveStraight();

        ShowTelemetry();

        do { // Drive until we reach the target distance
            UpdateCurrentPositions();
            _power = _PIDDriveDistance.performPID(_leftCurrentPosition);
            _correction = _PIDDriveStraight.performPID(_EHGyro.GetHeadingEH());

            ShowTelemetry();

            // TODO: We may need this.
//            if (_power + _correction >= -0.2) {
//                _power -= _correction;
//            }

            _WheelBackLeft.setPower(_power + _correction);
            _WheelBackRight.setPower(_power - _correction);
            _WheelFrontLeft.setPower(_power + _correction);
            _WheelFrontRight.setPower(_power - _correction);
            UpdateCurrentPositions();
        } while (!_PIDDriveDistance.onTarget());

        StopRobot();

        ShowTelemetry();

        // reset angle tracking on new heading.
        _EHGyro.ResetHeadingEH();

        sleep(_MILLS_TO_SLEEP);
    }

    public void Straight(Direction direction, double distanceInch, double power, double stopDistance) {
        sleep(100);

        int sign;
        stopDistanceReached = false;
        switch (direction) {
            case FORWARD:
                sign = 1; // Encoder values Decrease driving forward, hence a negative target distance.
                break;
            case BACKWARD:
                sign = -1;  // Encoder values Increase driving backward, hence a positive target distance.
                break;
            default:
                sign = 0;  // If we enter in the wrong direction, the robot won't move.
                break;

        }

        _targetDistance = Math.abs(InchesToDegrees(distanceInch)) * sign;
        _power = Math.abs(power) * sign;  // Power will be positive driving backward, and negative driving forward.
        ResetEncoders();

        // Update current and target distances
        UpdateCurrentPositions();
        UpdateTargetPositions(_targetDistance);

        // Set up parameters for driving in a straight line.
        ResetPIDDriveDistance();
        ResetPIDDriveStraight();

        ShowTelemetry();

        do { // Drive until we reach the target distance
            UpdateCurrentPositions();
            _power = _PIDDriveDistance.performPID(_leftCurrentPosition);
            _correction = 0;//_PIDDriveStraight.performPID(_EHGyro.GetHeadingEH());

//            telemetry.addData("Power = .12","Power = .12");
//            telemetry.update();
//
//            if (_power <= .12){
//                _power = .12;
//            }

            ShowTelemetry();

            // TODO: We may need this.
//            if (_power + _correction >= -0.2) {
//                _power -= _correction;
//            }

            _WheelBackLeft.setPower(_power + _correction);
            _WheelBackRight.setPower(_power - _correction);
            _WheelFrontLeft.setPower(_power + _correction);
            _WheelFrontRight.setPower(_power - _correction);
            UpdateCurrentPositions();

            if (rangeSensor.GetDistance() <= stopDistance)
                stopDistanceReached = true;

        } while (!_PIDDriveDistance.onTarget() || !stopDistanceReached);

        StopRobot();

        ShowTelemetry();

        // reset angle tracking on new heading.
        _EHGyro.ResetHeadingEH();

        sleep(_MILLS_TO_SLEEP);
    }

    private int _degrees;

    @Override
    public void Left(int degrees, double power) {
        sleep(100);
        _degrees = Math.abs(degrees); // Left turn is positive degrees
        _power = Math.abs(power);     // Given power will always be positive
        _EHGyro.ResetHeadingEH();

        ResetPIDRotate(false);

        ShowTelemetry();

        do {
            _power = _PIDRotate.performPID(_EHGyro.GetHeadingEH()); // power will be negative on a right turn
            ShowTelemetry();
            _WheelFrontLeft.setPower(_power);
            _WheelBackLeft.setPower(_power);
            _WheelFrontRight.setPower(-_power);
            _WheelBackRight.setPower(-_power);
        } while (!_PIDRotate.onTarget());

        StopRotation();

        ShowTelemetry();
        sleep(100);
        // reset angle tracking on new heading.
        _EHGyro.ResetHeadingEH();

        sleep(_MILLS_TO_SLEEP);

    }
//TODO: uncomment to work on right.

    @Override
    public void Right(int degrees, double power) {
        sleep(100);
        _degrees = Math.abs(degrees); // Right turn is negative degrees
        _power = Math.abs(power);      // Given power will always be positive
        _EHGyro.ResetHeadingEH();

        ResetPIDRotate(true);

        ShowTelemetry();

        do {
            _power = _PIDRotate.performPID(_EHGyro.GetHeadingEH()); // power will be negative on a right turn
            ShowTelemetry();
            _WheelFrontLeft.setPower(_power);
            _WheelBackLeft.setPower(_power);
            _WheelFrontRight.setPower(-_power);
            _WheelBackRight.setPower(-_power);
        } while (!_PIDRotate.onTarget());

        /*do {
            _power = _PIDRotate.performPID(_EHGyro.GetHeadingEH());
            ShowTelemetry();
            _WheelFrontLeft.setPower(-_power);
            _WheelBackLeft.setPower(-_power);
            _WheelFrontRight.setPower(_power);
            _WheelBackRight.setPower(_power);
        } while (!_PIDRotate.onTarget());*/

        StopRotation();

        ShowTelemetry();

        // reset angle tracking on new heading.
        _EHGyro.ResetHeadingEH();

        sleep(_MILLS_TO_SLEEP);
    }

    @Override
    public void BasicMotorControl(double right_stick_y) {
        _WheelFrontLeft.setPower(right_stick_y);
        _WheelFrontRight.setPower(right_stick_y);
        _WheelBackLeft.setPower(right_stick_y);
        _WheelBackRight.setPower(right_stick_y);
    }

    @Override
    public void StrafeRight(int SleepSeconds, double power) {

        _power = .4;

        _WheelFrontLeft.setPower(power);
        _WheelFrontRight.setPower(-power);
        _WheelBackLeft.setPower(-power);
        _WheelBackRight.setPower(power);

        long Seconds = SleepSeconds;
        sleep(Seconds);

        _WheelFrontLeft.setPower(0);
        _WheelFrontRight.setPower(0);
        _WheelBackLeft.setPower(0);
        _WheelBackRight.setPower(0);

    }

    @Override
    public void StrafeLeft(int SleepSeconds, double power) {


        _WheelFrontLeft.setPower(-power);
        _WheelFrontRight.setPower(power);
        _WheelBackLeft.setPower(power);
        _WheelBackRight.setPower(-power);

        long Seconds = SleepSeconds;
        sleep(Seconds);

        _WheelFrontLeft.setPower(0);
        _WheelFrontRight.setPower(0);
        _WheelBackLeft.setPower(0);
        _WheelBackRight.setPower(0);

    }

    @Override
    public void Intake(Direction direction, int IntakeRuntime, double power) {
        sleep(100);

        int sign;
        stopDistanceReached = false;
        switch (direction) {
            case FORWARD:
                sign = 1; // Encoder values Decrease driving forward, hence a negative target distance.
                break;
            case BACKWARD:
                sign = -1;  // Encoder values Increase driving backward, hence a positive target distance.
                break;
            default:
                sign = 0;  // If we enter in the wrong direction, the robot won't move.
                break;

        }
        _Intake.setPower(power*sign);
        sleep(IntakeRuntime);
        _Intake.setPower(0);

    }

    @Override
    public void ShowTelemetry() {
        telemetry.addData("PID", "--- PID Information ---");
        telemetry.addData("power", _power);

        telemetry.addData("min", _PIDRotate.m_minimumInput);
        telemetry.addData("max", _PIDRotate.m_maximumInput);
        telemetry.addData("set", _PIDRotate.m_setpoint);
        telemetry.addData("in", _PIDRotate.m_input);
        telemetry.addData("error", _PIDRotate.m_error);
        telemetry.addData("result", _PIDRotate.m_result);

        //telemetry.addData("target distance", _targetDistance);

        /*telemetry.addData("PID", "--- DriveDistance Information ---");
        telemetry.addData("CL", "Current Left  : " + _leftCurrentPosition);
        telemetry.addData("TL", "Target Left: " + _leftTargetPosition);
        telemetry.addData("CR", "Current Right  : " + _rightCurrentPosition);
        telemetry.addData("TR", "Target Right: " + _rightTargetPosition);
        telemetry.addData("target distance", _PIDDriveDistance.getSetpoint());
        telemetry.addData("error", _PIDDriveDistance.getError());
*/
        telemetry.addData("PID", "--- DriveStraight Information ---");
        telemetry.addData("heading", _EHGyro.GetHeadingEH());
        /*telemetry.addData("target heading", _PIDDriveStraight.getSetpoint());
        telemetry.addData("correction", _correction);*/

//        telemetry.addData("OD", "--- Odometer Information ---");
        telemetry.update();
    }

    private boolean onTarget() {
        return (Math.abs(_leftCurrentPosition) > Math.abs(_leftTargetPosition) || Math.abs(_rightCurrentPosition) > Math.abs(_rightTargetPosition));
    }

    private void StopRobot() {
        _WheelFrontLeft.setPower(0);
        _WheelBackLeft.setPower(0);
        _WheelFrontRight.setPower(0);
        _WheelBackRight.setPower(0);
    }

    private void UpdateCurrentPositions() {
        // Get Current positions in degrees
        _leftCurrentPosition = TicksToDegrees(_OdometerLeft.getCurrentPosition() * -1);
        _rightCurrentPosition = TicksToDegrees(_OdometerRight.getCurrentPosition());
    }

    private void UpdateTargetPositions(double distance) {
        // Get Target positions by adding current position and # of degrees to travel
        _leftTargetPosition = _leftCurrentPosition + _targetDistance;
        _rightTargetPosition = _rightCurrentPosition + _targetDistance;
    }

    private void ResetEncoders() {
        _OdometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _OdometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _OdometerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _OdometerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        _leftCurrentPosition = TicksToDegrees(_OdometerLeft.getCurrentPosition());
        _rightCurrentPosition = TicksToDegrees(_OdometerRight.getCurrentPosition());

        _leftTargetPosition = 0;
        _rightTargetPosition = 0;
    }

    private double CalcDistanceDegrees(double distanceInch) { return (TICKS_PER_INCH * distanceInch)/ DEGREES_PER_TICK; }

    private double InchesToTicks(double inches) {
        return inches * TICKS_PER_INCH;
    }

    private double TicksToDegrees(double ticks) {
        return ticks * DEGREES_PER_TICK;
    }

    private double InchesToDegrees(double inches) {
        return inches * TICKS_PER_INCH * DEGREES_PER_TICK;
    }

    private void ResetPIDDriveDistance(){

        _PIDDriveDistance.reset();

        // Proportional factor can be found by dividing the max desired pid output by
        // the setpoint or target. Here 30% power is divided by 90 degrees (.30 / 90)
        // to get a P factor of .003. This works for the robot we testing this code with.
        // Your robot may vary but this way finding P works well in most situations.
        double p = Math.abs(_power/ _targetDistance);

        // Integrative factor can be approximated by diving P by 100. Then you have to tune
        // this value until the robot turns, slows down and stops accurately and also does
        // not take too long to "home" in on the setpoint.
        double i = p / 175.0; // TODO: Figure this out

        _PIDDriveDistance.setPID(p, i, 0);

        _PIDDriveDistance.setInputRange(0, _targetDistance);
        _PIDDriveDistance.setSetpoint(_targetDistance);
        _PIDDriveDistance.setOutputRange(0, _power); // TODO: May need to change minimum output
        _PIDDriveDistance.setTolerance(1.0 / Math.abs(_targetDistance) * 100.0); // One degree as a percentage of the total degrees
        _PIDDriveDistance.enable();
    }

    private void ResetPIDDriveStraight(){
        _PIDDriveStraight.reset();

        double p = 0.05;

        // Integrative factor can be approximated by diving P by 100. Then you have to tune
        // this value until the robot turns, slows down and stops accurately and also does
        // not take too long to "home" in on the setpoint.
        double i = p / 175.0; // TODO: Try and find an integral that works well with driving straight.

        _PIDDriveStraight.setPID(p, 0, 0);

        _PIDDriveStraight.setSetpoint(0);
        _PIDDriveStraight.setOutputRange(0, _power);
        _PIDDriveStraight.setInputRange(-90, 90);
        _PIDDriveStraight.setContinuous(true);
        _PIDDriveStraight.enable();
    }


    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    private final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    /* Start pid controller. PID controller will monitor the turn angle with respect to the
     * target angle and reduce power as we approach the target angle. This is to prevent the
     * robots momentum from overshooting the turn after we turn off the power. The PID controller
     * reports onTarget() = true when the difference between turn angle and target angle is within
     * 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
     * dependant on the motor and gearing configuration, starting power, weight of the robot and the
     * on target tolerance. If the controller overshoots, it will reverse the sign of the output
     * turning the robot back toward the set point value. */
    private void ResetPIDRotate(boolean isRight){

        double minimuminput = _degrees - 5;
        double maximuminput = _degrees + 5;
        if (isRight) {
            /*minimuminput *= -1;
            maximuminput *= -1;*/
            _degrees *= -1;
        }
        _PIDRotate.reset();

        // Proportional factor can be found by dividing the max desired pid output by
        // the setpoint or target. Here 30% power is divided by 90 degrees (.30 / 90)
        // to get a P factor of .003. This works for the robot we testing this code with.
        // Your robot may vary but this way finding P works well in most situations.
        double p = Math.abs(_power/_degrees);

        // Integrative factor can be approximated by diving P by 100. Then you have to tune
        // this value until the robot turns, slows down and stops accurately and also does
        // not take too long to "home" in on the setpoint.
        double i = p / 175.0;

        _PIDRotate.setPID(p, i, 0);

        _PIDRotate.setInputRange(minimuminput, maximuminput);
        _PIDRotate.setSetpoint(_degrees);
        _PIDRotate.setOutputRange(.2, _power); // TODO: May need to change minimum output
        _PIDRotate.setTolerance(1.0 / Math.abs(_degrees) * 100.0); // One degree as a percentage of the total degrees
        _PIDRotate.enable();
    }

    // turn the motors off.
    private void StopRotation() {
        _WheelFrontLeft.setPower(0);
        _WheelBackLeft.setPower(0);
        _WheelFrontRight.setPower(0);
        _WheelBackRight.setPower(0);
    }
}
