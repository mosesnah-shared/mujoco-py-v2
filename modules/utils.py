# Define functions to be imported when used "import *"
# __all__  = [ "my_parser", "min_jerk_traj", "str2float", "get_model_prop", "get_data_prop", "get_length", "make_whip_downwards", "quaternion2euler", "print_vars" ] 

def min_jerk_traj( t: float, ti: float, tf: float, pi: float, pf: float, D: float ):
    """
        Descriptions
        ------------    
            Given t0 = Initial time, returning the 1D position and velocity data at time t0 of the minimum-jerk-trajectory ( current time )
            Note that the minimum-jerk-trajectory (MJT) remains at the initial (respectively, final) posture before (after) the movement.

        Parameters
        ----------
            (1) t: float (Non-negative)
                   Current Time of the Simulation

            (2) ti: float (Non-negative)
                   The initial time of the movement.

            (3) tf: float (Non-negative)
                   The final time of the movement.

            (4) pi: float (radian)
                   The initial posture of the MJT.

            (5) pf: float (radian)
                   The final posture of the MJT.

            (6) D: float (Positive)
                   The duration fo the movement.

        Arguments
        ---------
            (1) q0   - minimum jerk trajectory, position
                    q0   =  qi + ( qf - qi ) * ( 1O ( t/D )^3 - 15 ( t/D )^4 + 6 ( t/D )^5 )

            (2) dq0  - minimum jerk trajectory, velocity
                    dq0  = 1/D * ( qf - qi ) * ( 3O ( t/D )^2 - 60 ( t/D )^3 + 30 ( t/D )^4 )

            (3) ddq0 - minimum jerk trajectory, acceleration
                    ddq0 = 1/ ( D^2 ) * ( qf - qi ) * ( 6O ( t/D )^1 - 180 ( t/D )^2 + 120 ( t/D )^3 )

    """

    assert  t >=  0 and ti >= 0 and tf >= 0 and D >= 0
    assert tf >= ti

    if   t <= ti:
        pos = pi
        vel = 0
        acc = 0

    elif ti < t <= tf:
        tau = ( t - ti ) / D                                                # Normalized time
        pos =            pi + ( pf - pi ) * ( 10. * tau ** 3 -  15. * tau ** 4 +   6. * tau ** 5 )
        vel =        1. / D * ( pf - pi ) * ( 30. * tau ** 2 -  60. * tau ** 3 +  30. * tau ** 4 )
        acc = 1. / (D ** 2) * ( pf - pi ) * ( 60. * tau ** 1 - 180. * tau ** 2 + 120. * tau ** 3 )

    else:
        pos = pf
        vel = 0
        acc = 0

    return pos, vel, acc
