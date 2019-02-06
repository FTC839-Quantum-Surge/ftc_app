package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class ManualTargeting< T >
{
    protected DcMotor m_motor;
    protected int     m_nHomeEncPos = 0;

    protected T       m_eTargetPos;
    protected int     m_nTargetPosNumber;
    protected double  m_dTargetPower = 0;
    protected boolean m_bHoldPosition = false;


    // //////////////////////////////////////////////////////////////////////

    abstract protected int GetEncoderTargetValue( T eTarget );
    abstract protected T   GetNotTargetingValue ();

    // //////////////////////////////////////////////////////////////////////
    // override to implement limit switches
    // //////////////////////////////////////////////////////////////////////

    public boolean GetLimitTop   () { return false; }
    public boolean GetLimitBottom() { return false; }

    public double ScalePower( double dPower, int nAbsAmtToGo ) { return dPower; }
    public int    DeadZone  () {  return 10; }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    public ManualTargeting(DcMotor motor )
    {
        m_motor             = motor;
        m_nHomeEncPos       = m_motor.getCurrentPosition();
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    public int CurrentPos()
    {
        return  m_motor.getCurrentPosition();
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    public void SetPower( double dPower )
    {
        m_motor.setPower( dPower );
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////
    public void Stop( boolean bHoldPosition )
    {
        Stop( bHoldPosition, false );
    }

    public void Stop( boolean bHoldPosition, boolean bResetHome )
    {
        m_motor.setPower(0);

        if (bResetHome)
        {
            m_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m_nHomeEncPos = m_motor.getCurrentPosition();       // should always be 0!
        }

        if (!bHoldPosition)
        {
            m_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            m_eTargetPos = GetNotTargetingValue();

            return;
        }

        // Set Motor to use position mode to help maintain location.



    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    public void SetTarget( T eTarget, double dPower )
    {
        SetTarget( eTarget, dPower, false );
    }

    public void SetTarget( T eTarget, double dPower, boolean bHoldPosition )
    {
        m_bHoldPosition = bHoldPosition;
        m_dTargetPower  = dPower;

        if ( m_eTargetPos != GetNotTargetingValue() )
            Stop( false );

        m_eTargetPos = eTarget;
        m_nTargetPosNumber = m_nHomeEncPos + GetEncoderTargetValue( m_eTargetPos );

//        m_motor.setTargetPosition( nTarget);

        // ------------------------------------------------------------------
        // Turn On RUN_TO_POSITION
        // ------------------------------------------------------------------

//        m_motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
//        SetPower(  dPower );
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    public boolean PeriodicCheck( double dOverridePower )
    {
        double  dPower       = m_motor.getPower();
        boolean bLimitTop    = GetLimitTop();
        boolean bLimitBottom = GetLimitBottom();
        boolean bResult      = false;

        // ------------------------------------------------------------------
        // Is targeting a position?  If so, check to see if it reached the position.
        // ------------------------------------------------------------------

        if ( m_eTargetPos != GetNotTargetingValue() )
        {
            // Check Pos to see if target archieved

            int nAmtToGo = m_nTargetPosNumber - m_motor.getCurrentPosition();
            int nAbsAmtToGo = Math.abs( nAmtToGo );

            if ( nAbsAmtToGo > DeadZone() )
            {

                // Not there yet, keep moving

                // If not, determine direction to move

                double dDirection = (nAmtToGo < 0) ? -1.0 : 1.0;

                // Calculate power required (slow down as we approach)

                if (dPower == 0)
                    dPower = m_dTargetPower;

                dPower = ScalePower( Math.abs( dPower ), nAbsAmtToGo)  * dDirection;
                // TODO:
            }
            else
            {
                bResult = true;
                Stop( m_bHoldPosition );
            }
        }

        // ------------------------------------------------------------------
        // Is the user trying to manually move the stick, 0 power is okay if not targeting pos
        // ------------------------------------------------------------------

        if ((dOverridePower != 0) || (m_eTargetPos == GetNotTargetingValue() ))
        {
            // --------------------------------------------------------------
            // if Manual input, stop targeting
            // --------------------------------------------------------------

            if (m_eTargetPos != GetNotTargetingValue())
            {
                bResult = true;
                Stop( false );
            }

            dPower = dOverridePower;
        }

        // ------------------------------------------------------------------
        // Check limit switches
        // ------------------------------------------------------------------

        // ------------------------------------------------------------------
        // TODO: We don't know the direction of the motor when targeting, don't
        // check limit switch for now
        // ------------------------------------------------------------------

        //if (m_eTargetPos == GetNotTargetingValue() )
        {
            if ((bLimitTop == true) && (dPower > 0))
            {
                Stop( false  );
                return true;
            }
        }

        if ((bLimitBottom == true) && (dPower < 0))
        {
            Stop( false, true );
            return true;
        }

        SetPower( dPower );

        return bResult;
    }

}
