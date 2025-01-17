package Zakk;

import android.hardware.Sensor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "CodeGPT_Test", group = "Testing")
public class CodeGPT_Test extends OpMode {
    // Hardware
    private IMU imu;
    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;
    private DcMotor Arm;
    private DcMotor RotateArm;

    // Slow Mode Drive
    private boolean slowModeDriveOn = true;
    private boolean buttonSlowDriveIsPressed = false;
    private final double SLOW_DRIVE = 0.4;
    private final double FAST_DRIVE = 1.0;
    private double percentToSlowDrive = SLOW_DRIVE;

    // Sine Drive
    private boolean sineDriveOn = true;
    private boolean buttonSineIsPressed = false;
    private double modifyBySine = Math.sin(Math.PI / 4);

    // Encoder Constants
    private static final double TICKS_PER_REVOLUTION = 1120; // Example for an AndyMark NeveRest motor
    private static final double INCHES_PER_REVOLUTION = 4 * Math.PI; // Assuming a 4-inch wheel
    private static final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / INCHES_PER_REVOLUTION;
// commit
    @Override
    public void init() {
        WheelFrontLeft = hardwareMap.dcMotor.get("WheelFL");
        WheelFrontRight = hardwareMap.dcMotor.get("WheelFR");
        WheelBackLeft = hardwareMap.dcMotor.get("WheelBL");
        WheelBackRight = hardwareMap.dcMotor.get("WheelBR");
        Arm = hardwareMap.dcMotor.get("Arm");
        RotateArm = hardwareMap.dcMotor.get("RotateArm");

        resetEncoders();

        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        WheelFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm.setDirection(DcMotorSimple.Direction.FORWARD);
        RotateArm.setDirection(DcMotorSimple.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("I", "Initialization Complete! :D");
        telemetry.update();
    }

    @Override
    public void loop() {

        /*
         * Gamepad Controls
         */

        // Gamepad 1
        double oneLeftStickYPower = -gamepad1.left_stick_y;
        double oneLeftStickXPower = gamepad1.left_stick_x;
        double oneRightStickXPower = gamepad1.right_stick_x;
        double oneRightStickYPower = gamepad1.right_stick_y;
        boolean oneButtonA = gamepad1.a;
        boolean oneButtonB = gamepad1.b;
        boolean oneButtonX = gamepad1.x;
        boolean oneButtonY = gamepad1.y;
        boolean onePadUp = gamepad1.dpad_up;
        boolean onePadDown = gamepad1.dpad_down;
        boolean onePadLeft = gamepad1.dpad_left;
        boolean onePadRight = gamepad1.dpad_right;
        float oneTriggerLeft = gamepad1.left_trigger;
        float oneTriggerRight = gamepad1.right_trigger;
        boolean oneBumperLeft = gamepad1.left_bumper;
        boolean oneBumperRight = gamepad1.right_bumper;
        boolean oneBack = gamepad1.back;
        boolean oneStart = gamepad1.start;


        // Gamepad 2
        double twoLeftStickYPower = gamepad2.left_stick_y;
        double twoLeftStickXPower = gamepad2.left_stick_x;
        double twoRightStickXPower = gamepad2.right_stick_x;
        double twoRightStickYPower = gamepad2.right_stick_y;
        boolean twoButtonA = gamepad2.a;
        boolean twoButtonB = gamepad2.b;
        boolean twoButtonX = gamepad2.x;
        boolean twoButtonY = gamepad2.y;
        boolean twoPadUp = gamepad2.dpad_up;
        boolean twoPadDown = gamepad2.dpad_down;
        boolean twoPadLeft = gamepad2.dpad_left;
        boolean twoPadRight = gamepad2.dpad_right;
        float twoTriggerLeft = gamepad2.left_trigger;
        float twoTriggerRight = gamepad2.right_trigger;
        boolean twoBumperLeft = gamepad2.left_bumper;
        boolean twoBumperRight = gamepad2.right_bumper;
        boolean twoBack = gamepad2.back;
        boolean twoStart = gamepad2.start;

        // Reset encoders for Arm and RotateArm
        if (twoButtonA) {
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Arm Encoder", "Reset");
        }

        if (twoButtonB) {
            RotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("RotateArm Encoder", "Reset");
        }

        // Move the Arm and RotateArm while respecting encoder constraints
        controlArmWithEncoders(twoLeftStickYPower, twoLeftStickXPower);

        telemetry.update();
    }

    private void controlArmWithEncoders(double armPower, double rotatePower) {
        int armTicks = Arm.getCurrentPosition();
        int rotateTicks = RotateArm.getCurrentPosition();

        double armInches = armTicks / TICKS_PER_INCH;
        double rotateInches = rotateTicks / TICKS_PER_INCH;

        if (rotateInches <= 0.5 && armInches >= 38) {
            Arm.setPower(0); // Stop the Arm motor
        } else {
            Arm.setPower(armPower); // Allow movement otherwise
        }

        if (rotateInches > 2) {
            Arm.setPower(armPower); // Allow unrestricted movement
        }

        RotateArm.setPower(rotatePower);

        // Telemetry for debugging
        telemetry.addData("Arm Inches", armInches);
        telemetry.addData("Rotate Inches", rotateInches);
        telemetry.addData("Arm Power", armPower);
        telemetry.addData("Rotate Power", rotatePower);
    }

    private void resetEncoders() {
        WheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        WheelFrontLeft.setZeroPowerBehavior(behavior);
        WheelFrontRight.setZeroPowerBehavior(behavior);
        WheelBackLeft.setZeroPowerBehavior(behavior);
        WheelBackRight.setZeroPowerBehavior(behavior);
        Arm.setZeroPowerBehavior(behavior);
        RotateArm.setZeroPowerBehavior(behavior);
    }
}
