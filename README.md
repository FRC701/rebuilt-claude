# Rebuilt-Claude 

Rebuilt-Claude is a project where the code for the Rebuilt robot is developed
using Claude. Claude is used from the Claude application, not with Claude Code.
(Claude Code costs money).

There are a few goals to this project:

1. Learn more about how Claude (AI) can be used to generate code and build projects.
2. Learn how mentors can teach/mentor students to understand how AI can be used
   to generate code and build projects.
3. Learn how new information as a result of using Claude to develop the code. 
   Claude has access to a wide range of information including the history of code
   development for FRC, the entire library of WPILib functionality, as well as 
   fundamental and advanced Java techniques.

The project is developed subsystem by subsystem. Additonaly capabilities will be 
addedd once all of the subsystems have been included. The subsystems will be added
from simplest to most complex. Including:

1. Swerve (arguably the most complex, but Claude wanted to start there).
2. Agitator
3. Shooter
4. Roller
5. Intake
6. Climber
7. Vision
8. LEDs

The addition of each subsystem will add a section here with comments about learning
about WPILib, Claude, or other general subjects.

## Swerve

Swerve was an interesting system to develop. Swerve was developed using only WPILib. 
Swerve includes support for odometry, the IMU, as well as suppoprt for vision. Vision
is not yet added, but hooks are in place.

The architecture is uses a kind of Composite pattern (although not exactly). A Swerve 
Subsystem instantiates a Swerve Module instane for each swerve module. This is a useful
technique that can be used in other situations.

The swerve subsystem includes sysID support.

One command, TeleopSwerve is included. There are several useful features of TeleopSwerve

1. SlewRateLimiter to reduce slip on rapid changes
2. Input Squaring for better control at slow speeds

## Agitator

Not a lot of excitement for the Agigator. Claude chose to manage the two independent 
systems together rather than separate instances. This also happens later with the 
shooter. The agitator is less of a issue because we (almost) always want both motors
to run.

Commands AgitatorForward and AgitatorReverse are provided. These are simple commands
while not being instant commands.

## Shooter

The shooter had a couple of passes to get the system correct. Claude asked how the 
PID should be developed for the shooter. Adter a bit of discussion, we decided on
PID with Feedforward. (This is NOT what we are using right now.) This is a great 
way of managing the shooter PID.

The first pass included both shooters in one class with support for disabling 
them indepently. While correct, I suggested a system similar to the swerve module. 
Claude aggreed and generator new code that uses a composite-like system with where 
the shooter subsystem intantiates and controls Shooter Modules. 

Further discussion changed ther 14 parameter contructor (!) for the Shooter Module 
to intializing with a configuration structure. 

SysID support was added in order to develop the PID values for the shooter.

## Roller

## Intake

The intake uses the same module configuration architecture. Interestingly consistent, 
but seemingly unnecassary. 

Learned about MotionMagic. There is more tuning that could have been done to make MotionMagic
stay in place at the end. The complexity does not warrant the change.

Sepearate PID for deploy and retract helps with slop (maybe).

Compensate between deploy and retract using kG. Suggested using GravityTypeValue.Arm_Cosine! 
Didn't know this existed!

Use a timer (like we did) to support the bouncing. A bit different using rotation offsets instead
of specific positions. 

## Climber

## Vision

## LEDs



