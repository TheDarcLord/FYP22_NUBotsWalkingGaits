# ZMP - Zero Moment Point
### _Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point_
#### - Kajita, et al. 2003
`Shift + ⌘ + v`
https://en.wikipedia.org/wiki/Zero_moment_point
### __2.1__ 3D Linear Inverted Pendulum Mode and Zero-moment point
1. Constraint applied to a `3D Inverted Pendulum`, such that the mass moves along an `arbirary defined plane`.  
    __Arbitary Plane__: ${Z = k_xX + k_yY + z_c}$  
    ${z_c}$ is intersection of the `Z-plane` with coordinate system, when ${Z = 0}$ aka `the constraint plane`
2. Constraint applied such that the plane is horizontal to the `x-y plane`...     
  __IE:__ ${k_y = k_x =0}$
3. Dynamics of the Pendulum:  
    ${\ddot{y} = \frac{g}{z_c}y - \frac{1}{mz_c}\tau_x}$  
    ${\ddot{x} = \frac{g}{z_c}x + \frac{1}{mz_c}\tau_y}$  
    ...where:  
    $m$ = mass of the pendulum  
    $g$ = gravity  
    $\tau_x$ = $\Sigma$ input torques about $x$ axis  
    $\tau_y$ = $\Sigma$ input torques about $y$ axis
4. Additional Constraint:     
    ${\tau_xx + \tau_yy = 0}$   
    _This is an interesting one ..._
5. __ALSO ASSUMING WE'RE MOVING SO SLOW THAT JOINT TORQUES CAUSED BY SERVOS DON'T MATTER__  
    Sad... ? Extension on FYP ?     
    Dynamixels Suck.
6. 