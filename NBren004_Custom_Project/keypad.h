// Returns '\0' if no key pressed, else returns char '1', '2', ... '9', 'A', ...
// If multiple keys pressed, returns leftmost-topmost one
// Keypad must be connected to port A
/* Keypad arrangement
        PA4 PA5 PA6 PA7
   col  1   2   3   4
row
PC0 1   1 | 2 | 3 | A
PC1 2   4 | 5 | 6 | B
PC2 3   7 | 8 | 9 | C
PC3 4   * | 0 | # | D
*/

#define THE_PORT PORTC
#define THE_PIN PINC
// Must initialize DDR in main.c

unsigned char GetKeypadKey() {

	THE_PORT = 0xEF; // Enable col 4 with 0, disable others with 1’s
	asm("nop"); // add a delay to allow THE_PORT to stabilize before checking
	if (GetBit(THE_PIN,0)==0) { return('1'); }
	if (GetBit(THE_PIN,1)==0) { return('4'); }
	if (GetBit(THE_PIN,2)==0) { return('7'); }
	if (GetBit(THE_PIN,3)==0) { return('*'); }

	// Check keys in col 2
	THE_PORT = 0xDF; // Enable col 5 with 0, disable others with 1’s
	asm("nop"); // add a delay to allow THE_PORT to stabilize before checking
	if (GetBit(THE_PIN,0)==0) { return('2'); }
	if (GetBit(THE_PIN,1)==0) { return('5'); }
	if (GetBit(THE_PIN,2)==0) { return('8'); }
	if (GetBit(THE_PIN,3)==0) { return('0'); }


	// Check keys in col 3
	THE_PORT = 0xBF; // Enable col 6 with 0, disable others with 1’s
	asm("nop"); // add a delay to allow THE_PORT to stabilize before checking
	if (GetBit(THE_PIN,0)==0) { return('3'); }
	if (GetBit(THE_PIN,1)==0) { return('6'); }
	if (GetBit(THE_PIN,2)==0) { return('9'); }
	if (GetBit(THE_PIN,3)==0) { return('#'); }


	// Check keys in col 4	
    THE_PORT = 0x7F;
    asm("nop");
	if (GetBit(THE_PIN,0)==0) { return('A'); }
	if (GetBit(THE_PIN,1)==0) { return('B'); }
	if (GetBit(THE_PIN,2)==0) { return('C'); }
	if (GetBit(THE_PIN,3)==0) { return('D'); }


	return('\0'); // default value
}
