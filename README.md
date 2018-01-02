# PIC18F-Security-System-And-Traffic-Light
The makefile and asm file to correctly run a "security system" and traffic light simulation on a Microchip PIC18F. 
The program utilizes interrupts instead of polling.

# Security System
* The "correct" code is stored in the asm file, and can be modified
* User input for a code is read off of a switch input
* Once a momentary switch (button) is pushed down, two LEDs will go blank
* After 2 seconds of the button being held down the red LED will light if incorrect, green will light if correct.

# Traffic Light
* Can set up the sequence of lights for any amount of time you want in Sequence_Init block
* Traffic lights patterns will shine for 1 second for each input into the array and then loop
