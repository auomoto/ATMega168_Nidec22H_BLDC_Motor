ATMega168_Nidec22H_BLDC_Motor
=============================
This is the framework for an atmospheric dispersion corrector that requires two motors, two quadrature encoders, two home switches, and a deploy/retract mechanism. For now, we're using these surplus motors with integrated speed controls similar to the Faulhaber motors in the concept design.  

Next step is to add the LS7366 quadrature decoder/counters and use them to servo the position.

Or not. We might change to using a Faulhaber motor with a full motion controller so we won't even have to worry about an encoder.
