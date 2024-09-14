# Gear Box Calculations

We started with some basic assumptions about the length, weight, and required to move our arm. We assume that the arm will be 30.0" long, a weight of 10.0 lbs will be located at the end of the arm, and we'll want to rotate the arm at 15 RPM (which is equivalent to 1/4 rotation per second). We also assume that we'll need to move the arm from an initial position of 40 degrees off of horizontal, through 100 degrees of rotation, and ending at 40 degrees off the opposite horizon on the opposite side. For calculations, we introduced the following variables:

* $` l = 30 in `$ (arm length)
* $ w = 10 lbs $ (weight)
* $ \alpha = 40 degrees $ (minimum angle from horizontal)
* $ \Omega_a =  15 RPM  = 0.25 RPS$

The power needed to move the arm is variable. Positive power is needed when lifting the arm from its initial position to vertical. When the arm is vertical, the needed power is zero since the force of the weight is perpendicular to the direction of motion at that point. As the arm rotates toward its other limit, the vertical motion is in the same direction as the force of gravity, and thus power is required for braking. We can assume that the maximum power needed will occur when the arm is at an angle of $\alpha$. The power required at that point can be computed as:

$P_r = w * l * cos(\alpha) * \Omega_a$
$P_r = 10 lbs * 30 in * cos(40) * 0.25 RPS $
$P_r = 57.453 in * lbs / s $

We can convert $Pr$ to $Watts$ by:

$P_r = (57.453 in * lbs / s) * (.0254 m/in) * (4.448 N/lb)$
$P_r = 6.491 N * m / s$
$P_r = 6.491 W$

We plan to use the REV Robotics Neo Vortex motor. Specs for this motor can be found at: [https://www.revrobotics.com/rev-21-1652/](https://www.revrobotics.com/rev-21-1652/). Here, we see that peak power for this motor is 640 $W$, and power at 40 amps is 361 $W$. Both of these values are much greater than $P_r$, so we should have plenty of power for our application.

Since we believe we'll have plenty of power, we can concentrate on reducing the speed of the motor output to the speed we need to move the arm ($\Omega_a$). Looking at the power curve diagram for the motor (see link above), we find that 3000 $RPM$ is about in the middle of the power curve for this motor. Thus, we can say:

$\Omega_m = 3000 RPM = 50 RPS$

We now have enough information to compute our gear ratio ($A$):

$A = \Omega_m / \Omega_a$
$A = 50 RPS / 0.25 RPS$
$A = 200$

Thus, we need a gearbox that provides a reduction of 200:1.

