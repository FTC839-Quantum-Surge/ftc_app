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

package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.annotation.ElementType;

import fi.iki.elonen.NanoHTTPD;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="FindGold", group="FTC839")
//@Disabled
public class FindGold extends LinearOpMode {

    public enum  elementLocation
    {
        Unknown,
        Left,
        Center,
        Right
    }

    /* Declare OpMode members. */
    private Robot            robot       = new Robot( this );   // Use a Pushbot's hardware
    private ImageDetect      imageDetect = new ImageDetect();
    private ElapsedTime      runtime     = new ElapsedTime();

    //static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    //static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    //static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    //static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      //(WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, true);
        imageDetect.Initialize( hardwareMap, telemetry );

        // -------------------------------------------------------------------
        // Wait for the game to start (driver presses PLAY)
        // -------------------------------------------------------------------

        telemetry.addLine("Robot Initialization Complete");
        telemetry.addLine("Waiting for Start");
        telemetry.update();

        waitForStart();

        robot.OpModeStarted();

        // ------------------------------------------------------------------
        // Fold move to vertical
        // ------------------------------------------------------------------

        robot.Fold.SetTarget( Fold.PosEnum.Vertical, .25, true );

        // ------------------------------------------------------------------
        // Wait for Fold to stop
        // ------------------------------------------------------------------

        boolean bFoldComplete = false;

        while ( opModeIsActive()  && !bFoldComplete )
        {
            bFoldComplete = robot.Fold.PeriodicCheck( 0 );
        }

        // ------------------------------------------------------------------
        // Lower down from hanging position
        // ------------------------------------------------------------------

        robot.Lift.SetTarget( Lift.PosEnum.Hook, 1 );

        // ------------------------------------------------------------------
        // Wait for Lift to stop
        // ------------------------------------------------------------------

        boolean bLiftComplete = false;
        while ( opModeIsActive()  && !bLiftComplete )
        {
            bLiftComplete = robot.Lift.PeriodicCheck( 0 );
            robot.Fold.PeriodicCheck( 0 );
        }

        // ------------------------------------------------------------------
        // Open Claw
        // ------------------------------------------------------------------

        robot.OpenClaw();

        while( robot.IsClawOpen() == false) // Add timeout check|| runtime. = new ElapsedTime();
        {
            robot.Fold.PeriodicCheck(0);
        }

        sleep(1000);

        // ------------------------------------------------------------------
        // Lower Lift
        // ------------------------------------------------------------------

        robot.Lift.SetTarget( Lift.PosEnum.Bottom, 1 );

        // ------------------------------------------------------------------
        // Now that we are on the floor, detect the gold.
        // ------------------------------------------------------------------

        sleep( 500 );

        ImageDetect.GoldPositionInfo goldPos = imageDetect.DetectMinierals();

        // ------------------------------------------------------------------
        // Drive forward a little
        // ------------------------------------------------------------------

        robot.DriveDistance( DRIVE_SPEED,   2,   2, 90.0 );

        // ------------------------------------------------------------------
        // Determine which direction gold is...
        // If not found, look around for it
        // ------------------------------------------------------------------

        double dReturnAngle = goldPos.angle;

        elementLocation eLoc = elementLocation.Unknown;

        if (!goldPos.found()) {

            // Look to the right for gold.

            robot.rotate(-24, TURN_SPEED);
            dReturnAngle -=24;

            sleep(500);
            goldPos = imageDetect.DetectMinierals();
        }

        if (!goldPos.found()) {
            // Look to the left for gold.

            robot.rotate(50, TURN_SPEED);
            dReturnAngle += 50;
            sleep(500);
            goldPos = imageDetect.DetectMinierals();
        }

        if (!goldPos.found()) {
            // Not found anywhere return to pointing straight

            robot.rotate(-24, TURN_SPEED);
            dReturnAngle -= 50;
        }

        telemetry.addData( "Gold Found %b", goldPos.found());
        telemetry.addData( "Gold Found %d", goldPos.count  );
        telemetry.addData( "Gold Found %d", goldPos.angle  );
        telemetry.addData( "eLoc d%"      , eLoc           );

        telemetry.update();

        robot.rotate(  -goldPos.angle, TURN_SPEED );
        robot.DriveDistance( DRIVE_SPEED,   30,   30, 90.0 );

        robot.DriveDistance( DRIVE_SPEED,   -30,   -30, 90.0 );
        robot.rotate(  goldPos.angle - dReturnAngle, TURN_SPEED );

/*
        switch( eLoc )
        {
            case Right:
            {
                break;
            }

            case Center:
            {
                break;
            }

            case Left:
            {
                break;
            }
        }
*/
/*
        // ------------------------------------------------------------------
        // Drive forward a given distance
        // ------------------------------------------------------------------

        robot.DriveDistance( DRIVE_SPEED,  -30,  -30, 1.0 );
*/
        // ------------------------------------------------------------------
        // Fold move to folded
        // ------------------------------------------------------------------

        robot.Fold.SetTarget( Fold.PosEnum.Folded, .25, true );

        // ------------------------------------------------------------------
        // Set motors to zero
        // ------------------------------------------------------------------

        robot.SetDrivePower( 0, 0 );

        robot.OpModeStopped();
        imageDetect.Stop();

        telemetry.addData("FindGold", "Complete");
        telemetry.update();
    }
}
