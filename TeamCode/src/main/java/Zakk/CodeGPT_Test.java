package Zakk;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "CodeGPT_Test", group = "Testing")
public class CodeGPT_Test extends OpMode {
    /*
     * Declare Hardware
     */

    // IMU
    private IMU imu;

    // Wheels
    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;

    // Arm
    private DcMotor Arm;
    private DcMotor RotateArm;

    // SlowMode Drive
    private boolean slowModeDriveOn = true;
    private boolean buttonSlowDriveIsPressed = false;
    private final double SLOW_DRIVE = 0.4;
    private final double FAST_DRIVE = 1.0; // 0.9;
    private double percentToSlowDrive = SLOW_DRIVE;

    // SineDrive
    private boolean sineDriveOn = true;
    private boolean buttonSineIsPressed = false;
    private double modifyBySine = Math.sin(Math.PI / 4);

    // Encoder Constants
    private static final double TICKS_PER_REVOLUTION = 1120; // Example for an AndyMark NeveRest motor
    private static final double INCHES_PER_REVOLUTION = 4 * Math.PI; // Assuming a 4-inch wheel
    private static final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / INCHES_PER_REVOLUTION;

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
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);
        RotateArm.setDirection(DcMotorSimple.Direction.FORWARD);

        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        double twoLeftStickYPower = -gamepad2.left_stick_y;
        double twoLeftStickXPower = gamepad2.left_stick_x;
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

        //double ArmPower = 0;


        /*
         * Reset Encoders
         */
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

        /*
         * Do Stuff Here!
         */

        ToggleSlowModeDrive(oneButtonA);
        ProMotorControl(oneLeftStickYPower, oneLeftStickXPower, oneRightStickXPower);

        //IsArmVertical(ArmPower);

        controlArmWithEncoders(twoLeftStickYPower, twoLeftStickXPower);

        telemetry.update();
    }

    /*
     * Methods
     */

    private void ProMotorControl(double left_stick_y, double left_stick_x, double right_stick_x) {
        double powerLeftY = left_stick_y;   // DRIVE : Backward -1 <---> 1 Forward
        double powerLeftX = -left_stick_x * -1; // STRAFE:     Left -1 <---> 1 Right
        double powerRightX = right_stick_x; // ROTATE:     Left -1 <---> 1 Right

        double r = Math.hypot(powerLeftX, powerLeftY);
        double robotAngle = Math.atan2(powerLeftY, powerLeftX) - Math.PI / 4;
        double leftX = powerRightX;
        final double v1 = r * Math.cos(robotAngle) / modifyBySine + leftX;
        final double v2 = r * Math.sin(robotAngle) / modifyBySine - leftX;
        final double v3 = r * Math.sin(robotAngle) / modifyBySine + leftX;
        final double v4 = r * Math.cos(robotAngle) / modifyBySine - leftX;

        WheelFrontLeft.setPower(v1 * percentToSlowDrive);
        WheelFrontRight.setPower(v2 * percentToSlowDrive);
        WheelBackLeft.setPower(v3 * percentToSlowDrive);
        WheelBackRight.setPower(v4 * percentToSlowDrive);

        telemetry.addData("Wheel Front Left", v1 * percentToSlowDrive);
        telemetry.addData("Wheel Front Right", v2 * percentToSlowDrive);
        telemetry.addData("Wheel Back Left", v3 * percentToSlowDrive);
        telemetry.addData("Wheel Back Right", v4 * percentToSlowDrive);
    }

    private void controlArmWithEncoders(double armPower, double rotatePower) {
        int armTicks = Arm.getCurrentPosition();
        int rotateTicks = RotateArm.getCurrentPosition();

        double armInches = armTicks / TICKS_PER_INCH;
        double rotateInches = rotateTicks / TICKS_PER_INCH;

        // Allow Arm to retract even if RotateArm < 2 inches
        if (rotateInches < -30 && armInches >= -19) {
            if (armPower < 0) { // Retraction
                Arm.setPower(armPower);
            } else {
                Arm.setPower(armPower*0); // Nullify forward movement
            }
        } else {
            // Normal behavior when RotateArm >= 2 inches
            if (rotateInches >= -30.1) {
                Arm.setPower(armPower);
            }
        }

        RotateArm.setPower(rotatePower);

        // Telemetry for debugging
        telemetry.addData("Arm Inches", armInches);
        telemetry.addData("Rotate Inches", rotateInches);
        telemetry.addData("Arm Power", armPower);
        telemetry.addData("Rotate Power", rotatePower);
    }


    private void ToggleSlowModeDrive(boolean button) {
        if (button && !buttonSlowDriveIsPressed) {
            buttonSlowDriveIsPressed = true;
            slowModeDriveOn = !slowModeDriveOn;
        }
        if (!button) {
            buttonSlowDriveIsPressed = false;
        }

        if (slowModeDriveOn) {
            percentToSlowDrive = SLOW_DRIVE;
            telemetry.addData("Drive Mode", "Slow: " + percentToSlowDrive + "% Power");
        } else {
            percentToSlowDrive = FAST_DRIVE;
            telemetry.addData("Drive Mode", "Fast: " + percentToSlowDrive + "% Power");
        }
    }

//    private void IsArmVertical(double powah) {
//        int armTicks = Arm.getCurrentPosition();
//        int rotateTicks = RotateArm.getCurrentPosition();
//
//        double armInches = armTicks / TICKS_PER_INCH;
//        double rotateInches = rotateTicks / TICKS_PER_INCH;
//
//        if (rotateInches < -30 && armInches >= -18){
//            powah = 0;
//        } else {
//
//        }
//    }

    private void resetEncoders() {
        WheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
