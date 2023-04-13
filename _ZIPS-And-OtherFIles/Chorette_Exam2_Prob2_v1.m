%{
Nicholas Chorette
3/31/2023
Aircraft Simulation : Exam 2 

Problem #2
%}

s = tf('s'); %transfer function 
p = -7/( s*s+10*s+7);
step(p)

%{
Short Analysis: 

The plot is critically damped because... to simplify things, there is no
"s" in the numerator, aka, the top half of the equation is constant, thus
critical damping is allowed to occur. 

Because it is a negative constant, the responce decreases over time until
it approaches an amplitude of -1. 


%}