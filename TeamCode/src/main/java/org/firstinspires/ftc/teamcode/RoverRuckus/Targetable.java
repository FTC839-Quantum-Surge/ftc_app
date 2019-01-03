package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Targetable< T >
{
    protected DcMotor m_motor;
    protected int     m_nHomeEncPos = 0;

    protected T       m_eTargetPos;
    protected boolean m_bHoldPosition = false;

    // //////////////////////////////////////////////////////////////////////

    abstract protected int GetEncoderTargetValue( T eTarget );
    abstract protected T   GetNotTargetingValue ();

    // //////////////////////////////////////////////////////////////////////
    // override to implement limit switches
    // //////////////////////////////////////////////////////////////////////

    public boolean GetLimitTop   () { return false; }
    public boolean GetLimitBottom() { return false; }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    public Targetable( DcMotor motor )
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

    public void Stop()
    {
        m_motor.setPower( 0 );
        m_motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

        m_eTargetPos = GetNotTargetingValue();
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

        if ( m_eTargetPos != GetNotTargetingValue() )
            Stop();

        m_eTargetPos = eTarget;
        int nTarget = m_nHomeEncPos + GetEncoderTargetValue( m_eTargetPos );

        m_motor.setTargetPosition( nTarget);

        // ------------------------------------------------------------------
        // Turn On RUN_TO_POSITION
        // ------------------------------------------------------------------

        m_motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
        SetPower(  dPower );
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
            if ( m_motor.isBusy() == false)
            {
                if (!m_bHoldPosition)
                    Stop();

                bResult = true;
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
                Stop();

            dPower = dOverridePower;
        }

        // ------------------------------------------------------------------
        // Check limit switches
        // ------------------------------------------------------------------

        // ------------------------------------------------------------------
        // TODO: We don't know the direction of the motor when targeting, don't
        // check limit switch for now
        // ------------------------------------------------------------------

        if (m_eTargetPos == GetNotTargetingValue() )
        {
            if ((bLimitTop == false) && (dPower > 0))
            {
                Stop();
                return true;
            }
        }

        if ((bLimitBottom == false) && (dPower < 0))
        {
            Stop();
            return true;
        }

        SetPower( dPower );

        return bResult;
    }

}
