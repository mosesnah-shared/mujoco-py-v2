# You need a class structure for this case 

def first_order_joint_space_impedance( Kq, Bq, q, dq, q0, dq0 ):
    """
        Descriptions
        ------------
            First-order Joint-space Impedance Controller

        Parameters
        ----------
            (1) model: mujoco Model 
                    The basic mujoco model class for mujoco

            (2) data: mujoco Data
                    The basic mujoco data class for mujoco            
            
            (3) Kq: Float Matrix ( nq x nq )
                    The Joint-stiffness matrix
            
            (4) Bq: Float Matrix ( nq x nq )
                    The Joint-damping matrix
            
            (5) q: Float Array ( 1 x nq )
                    The joint-displacement of the nq-DOF robot
            
            (6) dq: Float Array ( 1 x nq ) 
                    The joint-velocity vector of the nq-DOF robot

            (7) q0: Float Array ( 1 x nq ) 
                    The nominal joint posture of the nq-DOF robot
            
            (8) dq0: Float Array ( 1 x nq )                      
                    The nominal joint velocity of the nq-DOF robot   

        Returns
        ----------                        
            (1) tau: Float Array ( 1 x nq )
                    tau = Kq (q0 - q ) + Bq( q0 - q )

    """
    

    

def first_order_task_space_position_impedance( Kp, Bp, p, dp, p0, dp0 ):
    """
        Descriptions
        ------------
            First-order Joint-space Impedance Controller

        Parameters
        ----------
            (1) model: mujoco Model 
                    The basic mujoco model class for mujoco

            (2) data: mujoco Data
                    The basic mujoco data class for mujoco            
            
            (3) Kp:
            
            (4) Bp:
            
            (5) p:
            
            (6) dp: 

            (7) p0:
            
            (8) dp0:                        

        Returns
        ----------                            
    
    """
    NotImplementedError( )