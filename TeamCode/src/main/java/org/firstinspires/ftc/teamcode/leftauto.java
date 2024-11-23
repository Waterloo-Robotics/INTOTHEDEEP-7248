/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="LeftStrafeAuto", group="Robot")

public class leftauto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    TouchSensor HangerTop = null;
//    TouchSensor HangerBase = null;



    static final double     FORWARD_SPEED = 0.3;
    static final double     REVERSE_SPEED = 0.3;
    static final double     TURN_SPEED    = 0.7;
    static final double     SPIN_SPEED = 0.4;




    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LD");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bldr");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RD");
        rightBackDrive = hardwareMap.get(DcMotor.class, "brdr");
        arm = hardwareMap.get(DcMotor.class, "arm");
        HangerTop = hardwareMap.get(TouchSensor.class, "HTLimit");
//        HangerBase = hardwareMap.get(TouchSensor.class, "HBLimit");
//        HangerTop = hardwareMap.get(TouchSensor.class, "HTLimit");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //ru
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // move forward off of the wall
        drive_me(-0.5,-0.5,-0.5,-0.5,0.09);
        // wait
        drive_me(0,0,0,0, 0.1);
        // first block push to the left into scoring zone
        drive_me(-0.5, 0.5, -0.5, 0.5, 0.85);
        // wait 0.2 seconds
        drive_me(0,0,0,0, 0.2);
        // move to the right
        drive_me(0.5, -0.5, 0.5, -0.5, 0.69);
        // forward to slightly pass yellow block 1
        drive_me(-0.5,-0.5,-0.5,-0.5,1.57);
        // stop 0.5 seconds
        drive_me(0,0,0,0, 0.5);
        // move left to grab yellow block 1
        drive_me(-0.5, 0.5, -0.5, 0.5,0.48);
        // backwards to put yellow 1 in score zone
        drive_me(0,0,0,0, 0.5);
        // move forward to align with yellow 2
        drive_me(0.5, 0.5, 0.5, 0.5, 1.57);
        // wait 2 seconds
        drive_me(0,0,0,0, 0.2);
        // go forward to slightly pass yellow block 2
        drive_me(-0.5,-0.5,-0.5,-0.5,1.5);
        // move left to align and grab yellow block 2
        drive_me(-0.5, 0.5, -0.5, 0.5, 0.39);
        // move yellow block 2 backwards into scoring zone
        drive_me(0.5, 0.5, 0.5, 0.5, 1.61);
        // pull forward to slightly pass yellow block 3
        drive_me(-0.5,-0.5,-0.5,-0.5,1.55);
        // stop for 0.2
        drive_me(0,0,0,0, 0.2);
        // move left align and get yellow 3
        drive_me(-0.5, 0.5, -0.5, 0.5, 0.5);
        // move backwards to score yellow 3
        drive_me(0.5, 0.5, 0.5, 0.5, 1.41);
        // strafe left slightly
        drive_me(-0.5, 0.5, -0.5, 0.5, 0.5);
        // stop for 0.2
        drive_me(0,0,0,0, 0.2);
        // go forward
        drive_me(-0.5,-0.5,-0.5,-0.5,1.25);
        //strafe right so we don't clip the wall while turning
        drive_me(0.5, -0.5, 0.5, -0.5, 0.4);
        // turn to face the bar
        drive_me(0.5,0.5,-0.5,-0.5,0.76);
//        // move up to bar
//        drive_me(-0.5,-0.5,-0.5,-0.5,1);
//        // extend the arm
//        arm.setTargetPosition(-10135);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (opModeIsActive() && arm.getTargetPosition() >= -10147 && !HangerTop.isPressed()) {
//            arm.setPower(.75);
//        }
//        arm.setPower(0);



        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
    public void drive_me(double front_right, double back_right, double back_left, double front_left, double seconds){
        this.rightFrontDrive.setPower(front_right);
        this.rightBackDrive.setPower(back_right);
        this.leftBackDrive.setPower(back_left);
        this.leftFrontDrive.setPower(front_left);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < seconds)){

        }
        this.rightFrontDrive.setPower(0);
        this.rightBackDrive.setPower(0);
        this.leftBackDrive.setPower(0);
        this.leftFrontDrive.setPower(0);
    }
}