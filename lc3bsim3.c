/*
    William Peng
    wp4872
*/

/***************************************************************/
/*                                                             */
/*   LC-3b Simulator                                           */
/*                                                             */
/*   EE 460N                                                   */
/*   The University of Texas at Austin                         */
/*                                                             */
/***************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file                      */
/*         isaprogram   LC-3b machine language program file    */
/*                                                             */
/***************************************************************/

/***************************************************************/
/* These are the functions you'll have to write.               */
/***************************************************************/

void eval_micro_sequencer();
void cycle_memory();
void eval_bus_drivers();
void drive_bus();
void latch_datapath_values();


/***************************************************************/
/* Helper Function Declarations                                */
/***************************************************************/

int SEXT(int num, int signBit);
void setCC(int num);


/***************************************************************/
/* A couple of useful definitions.                             */
/***************************************************************/
#define FALSE 0
#define TRUE  1

/***************************************************************/
/* Use this to avoid overflowing 16 bits on the bus.           */
/***************************************************************/
#define Low16bits(x) ((x) & 0xFFFF)

/***************************************************************/
/* Definition of the control store layout.                     */
/***************************************************************/
#define CONTROL_STORE_ROWS 64
#define INITIAL_STATE_NUMBER 18

/***************************************************************/
/* Definition of bit order in control store word.              */
/***************************************************************/
enum CS_BITS {                                                  
    IRD,
    COND1, COND0,
    J5, J4, J3, J2, J1, J0,
    LD_MAR,
    LD_MDR,
    LD_IR,
    LD_BEN,
    LD_REG,
    LD_CC,
    LD_PC,
    GATE_PC,
    GATE_MDR,
    GATE_ALU,
    GATE_MARMUX,
    GATE_SHF,
    PCMUX1, PCMUX0,
    DRMUX,
    SR1MUX,
    ADDR1MUX,
    ADDR2MUX1, ADDR2MUX0,
    MARMUX,
    ALUK1, ALUK0,
    MIO_EN,
    R_W,
    DATA_SIZE,
    LSHF1,
    CONTROL_STORE_BITS
} CS_BITS;

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x)           { return(x[IRD]); }
int GetCOND(int *x)          { return((x[COND1] << 1) + x[COND0]); }
int GetJ(int *x)             { return((x[J5] << 5) + (x[J4] << 4) +
				      (x[J3] << 3) + (x[J2] << 2) +
				      (x[J1] << 1) + x[J0]); }
int GetLD_MAR(int *x)        { return(x[LD_MAR]); }
int GetLD_MDR(int *x)        { return(x[LD_MDR]); }
int GetLD_IR(int *x)         { return(x[LD_IR]); }
int GetLD_BEN(int *x)        { return(x[LD_BEN]); }
int GetLD_REG(int *x)        { return(x[LD_REG]); }
int GetLD_CC(int *x)         { return(x[LD_CC]); }
int GetLD_PC(int *x)         { return(x[LD_PC]); }
int GetGATE_PC(int *x)       { return(x[GATE_PC]); }
int GetGATE_MDR(int *x)      { return(x[GATE_MDR]); }
int GetGATE_ALU(int *x)      { return(x[GATE_ALU]); }
int GetGATE_MARMUX(int *x)   { return(x[GATE_MARMUX]); }
int GetGATE_SHF(int *x)      { return(x[GATE_SHF]); }
int GetPCMUX(int *x)         { return((x[PCMUX1] << 1) + x[PCMUX0]); }
int GetDRMUX(int *x)         { return(x[DRMUX]); }
int GetSR1MUX(int *x)        { return(x[SR1MUX]); }
int GetADDR1MUX(int *x)      { return(x[ADDR1MUX]); }
int GetADDR2MUX(int *x)      { return((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x)        { return(x[MARMUX]); }
int GetALUK(int *x)          { return((x[ALUK1] << 1) + x[ALUK0]); }
int GetMIO_EN(int *x)        { return(x[MIO_EN]); }
int GetR_W(int *x)           { return(x[R_W]); }
int GetDATA_SIZE(int *x)     { return(x[DATA_SIZE]); } 
int GetLSHF1(int *x)         { return(x[LSHF1]); }

/***************************************************************/
/* The control store rom.                                      */
/***************************************************************/
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];

/***************************************************************/
/* Main memory.                                                */
/***************************************************************/
/* MEMORY[A][0] stores the least significant byte of word at word address A
   MEMORY[A][1] stores the most significant byte of word at word address A 
   There are two write enable signals, one for each byte. WE0 is used for 
   the least significant byte of a word. WE1 is used for the most significant 
   byte of a word. */

#define WORDS_IN_MEM    0x08000 
#define MEM_CYCLES      5
int MEMORY[WORDS_IN_MEM][2];

/***************************************************************/

/***************************************************************/

/***************************************************************/
/* LC-3b State info.                                           */
/***************************************************************/
#define LC_3b_REGS 8

int RUN_BIT;	/* run bit */
int BUS;	/* value of the bus */

typedef struct System_Latches_Struct{

int PC,		/* program counter */
    MDR,	/* memory data register */
    MAR,	/* memory address register */
    IR,		/* instruction register */
    N,		/* n condition bit */
    Z,		/* z condition bit */
    P,		/* p condition bit */
    BEN;        /* ben register */

int READY;	/* ready bit */
  /* The ready bit is also latched as you dont want the memory system to assert it 
     at a bad point in the cycle*/

int REGS[LC_3b_REGS]; /* register file. */

int MICROINSTRUCTION[CONTROL_STORE_BITS]; /* The microintruction */

int STATE_NUMBER; /* Current State Number - Provided for debugging */ 
} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/***************************************************************/
/* A cycle counter.                                            */
/***************************************************************/
int CYCLE_COUNT;

/***************************************************************/
/*                                                             */
/* Procedure : help                                            */
/*                                                             */
/* Purpose   : Print out a list of commands.                   */
/*                                                             */
/***************************************************************/
void help() {                                                    
    printf("----------------LC-3bSIM Help-------------------------\n");
    printf("go               -  run program to completion       \n");
    printf("run n            -  execute program for n cycles    \n");
    printf("mdump low high   -  dump memory from low to high    \n");
    printf("rdump            -  dump the register & bus values  \n");
    printf("?                -  display this help menu          \n");
    printf("quit             -  exit the program                \n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : cycle                                           */
/*                                                             */
/* Purpose   : Execute a cycle                                 */
/*                                                             */
/***************************************************************/
void cycle() {                                                

  eval_micro_sequencer();   
  cycle_memory();
  eval_bus_drivers();
  drive_bus();
  latch_datapath_values();

  CURRENT_LATCHES = NEXT_LATCHES;

  CYCLE_COUNT++;
}

/***************************************************************/
/*                                                             */
/* Procedure : run n                                           */
/*                                                             */
/* Purpose   : Simulate the LC-3b for n cycles.                 */
/*                                                             */
/***************************************************************/
void run(int num_cycles) {                                      
    int i;

    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating for %d cycles...\n\n", num_cycles);
    for (i = 0; i < num_cycles; i++) {
	if (CURRENT_LATCHES.PC == 0x0000) {
	    RUN_BIT = FALSE;
	    printf("Simulator halted\n\n");
	    break;
	}
	cycle();
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : go                                              */
/*                                                             */
/* Purpose   : Simulate the LC-3b until HALTed.                 */
/*                                                             */
/***************************************************************/
void go() {                                                     
    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating...\n\n");
    while (CURRENT_LATCHES.PC != 0x0000)
	cycle();
    RUN_BIT = FALSE;
    printf("Simulator halted\n\n");
}

/***************************************************************/ 
/*                                                             */
/* Procedure : mdump                                           */
/*                                                             */
/* Purpose   : Dump a word-aligned region of memory to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void mdump(FILE * dumpsim_file, int start, int stop) {          
    int address; /* this is a byte address */

    printf("\nMemory content [0x%.4x..0x%.4x] :\n", start, stop);
    printf("-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	printf("  0x%.4x (%d) : 0x%.2x%.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    printf("\n");

    /* dump the memory contents into the dumpsim file */
    fprintf(dumpsim_file, "\nMemory content [0x%.4x..0x%.4x] :\n", start, stop);
    fprintf(dumpsim_file, "-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	fprintf(dumpsim_file, " 0x%.4x (%d) : 0x%.2x%.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : rdump                                           */
/*                                                             */
/* Purpose   : Dump current register and bus values to the     */   
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void rdump(FILE * dumpsim_file) {                               
    int k; 

    printf("\nCurrent register/bus values :\n");
    printf("-------------------------------------\n");
    printf("Cycle Count  : %d\n", CYCLE_COUNT);
    printf("PC           : 0x%.4x\n", CURRENT_LATCHES.PC);
    printf("IR           : 0x%.4x\n", CURRENT_LATCHES.IR);
    printf("STATE_NUMBER : 0x%.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    printf("BUS          : 0x%.4x\n", BUS);
    printf("MDR          : 0x%.4x\n", CURRENT_LATCHES.MDR);
    printf("MAR          : 0x%.4x\n", CURRENT_LATCHES.MAR);
    printf("CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    printf("Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	printf("%d: 0x%.4x\n", k, CURRENT_LATCHES.REGS[k]);
    printf("\n");

    /* dump the state information into the dumpsim file */
    fprintf(dumpsim_file, "\nCurrent register/bus values :\n");
    fprintf(dumpsim_file, "-------------------------------------\n");
    fprintf(dumpsim_file, "Cycle Count  : %d\n", CYCLE_COUNT);
    fprintf(dumpsim_file, "PC           : 0x%.4x\n", CURRENT_LATCHES.PC);
    fprintf(dumpsim_file, "IR           : 0x%.4x\n", CURRENT_LATCHES.IR);
    fprintf(dumpsim_file, "STATE_NUMBER : 0x%.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    fprintf(dumpsim_file, "BUS          : 0x%.4x\n", BUS);
    fprintf(dumpsim_file, "MDR          : 0x%.4x\n", CURRENT_LATCHES.MDR);
    fprintf(dumpsim_file, "MAR          : 0x%.4x\n", CURRENT_LATCHES.MAR);
    fprintf(dumpsim_file, "CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    fprintf(dumpsim_file, "Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	fprintf(dumpsim_file, "%d: 0x%.4x\n", k, CURRENT_LATCHES.REGS[k]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : get_command                                     */
/*                                                             */
/* Purpose   : Read a command from standard input.             */  
/*                                                             */
/***************************************************************/
void get_command(FILE * dumpsim_file) {                         
    char buffer[20];
    int start, stop, cycles;

    printf("LC-3b-SIM> ");

    scanf("%s", buffer);
    printf("\n");

    switch(buffer[0]) {
    case 'G':
    case 'g':
	go();
	break;

    case 'M':
    case 'm':
	scanf("%i %i", &start, &stop);
	mdump(dumpsim_file, start, stop);
	break;

    case '?':
	help();
	break;
    case 'Q':
    case 'q':
	printf("Bye.\n");
	exit(0);

    case 'R':
    case 'r':
	if (buffer[1] == 'd' || buffer[1] == 'D')
	    rdump(dumpsim_file);
	else {
	    scanf("%d", &cycles);
	    run(cycles);
	}
	break;

    default:
	printf("Invalid Command\n");
	break;
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : init_control_store                              */
/*                                                             */
/* Purpose   : Load microprogram into control store ROM        */ 
/*                                                             */
/***************************************************************/
void init_control_store(char *ucode_filename) {                 
    FILE *ucode;
    int i, j, index;
    char line[200];

    printf("Loading Control Store from file: %s\n", ucode_filename);

    /* Open the micro-code file. */
    if ((ucode = fopen(ucode_filename, "r")) == NULL) {
	printf("Error: Can't open micro-code file %s\n", ucode_filename);
	exit(-1);
    }

    /* Read a line for each row in the control store. */
    for(i = 0; i < CONTROL_STORE_ROWS; i++) {
	if (fscanf(ucode, "%[^\n]\n", line) == EOF) {
	    printf("Error: Too few lines (%d) in micro-code file: %s\n",
		   i, ucode_filename);
	    exit(-1);
	}

	/* Put in bits one at a time. */
	index = 0;

	for (j = 0; j < CONTROL_STORE_BITS; j++) {
	    /* Needs to find enough bits in line. */
	    if (line[index] == '\0') {
		printf("Error: Too few control bits in micro-code file: %s\nLine: %d\n",
		       ucode_filename, i);
		exit(-1);
	    }
	    if (line[index] != '0' && line[index] != '1') {
		printf("Error: Unknown value in micro-code file: %s\nLine: %d, Bit: %d\n",
		       ucode_filename, i, j);
		exit(-1);
	    }

	    /* Set the bit in the Control Store. */
	    CONTROL_STORE[i][j] = (line[index] == '0') ? 0:1;
	    index++;
	}

	/* Warn about extra bits in line. */
	if (line[index] != '\0')
	    printf("Warning: Extra bit(s) in control store file %s. Line: %d\n",
		   ucode_filename, i);
    }
    printf("\n");
}

/************************************************************/
/*                                                          */
/* Procedure : init_memory                                  */
/*                                                          */
/* Purpose   : Zero out the memory array                    */
/*                                                          */
/************************************************************/
void init_memory() {                                           
    int i;

    for (i=0; i < WORDS_IN_MEM; i++) {
	MEMORY[i][0] = 0;
	MEMORY[i][1] = 0;
    }
}

/**************************************************************/
/*                                                            */
/* Procedure : load_program                                   */
/*                                                            */
/* Purpose   : Load program and service routines into mem.    */
/*                                                            */
/**************************************************************/
void load_program(char *program_filename) {                   
    FILE * prog;
    int ii, word, program_base;

    /* Open program file. */
    prog = fopen(program_filename, "r");
    if (prog == NULL) {
	printf("Error: Can't open program file %s\n", program_filename);
	exit(-1);
    }

    /* Read in the program. */
    if (fscanf(prog, "%x\n", &word) != EOF)
	program_base = word >> 1;
    else {
	printf("Error: Program file is empty\n");
	exit(-1);
    }

    ii = 0;
    while (fscanf(prog, "%x\n", &word) != EOF) {
	/* Make sure it fits. */
	if (program_base + ii >= WORDS_IN_MEM) {
	    printf("Error: Program file %s is too long to fit in memory. %x\n",
		   program_filename, ii);
	    exit(-1);
	}

	/* Write the word to memory array. */
	MEMORY[program_base + ii][0] = word & 0x00FF;
	MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;
	ii++;
    }

    if (CURRENT_LATCHES.PC == 0) CURRENT_LATCHES.PC = (program_base << 1);

    printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */ 
/*             and set up initial state of the machine.        */
/*                                                             */
/***************************************************************/
void initialize(char *ucode_filename, char *program_filename, int num_prog_files) { 
    int i;
    init_control_store(ucode_filename);

    init_memory();
    for ( i = 0; i < num_prog_files; i++ ) {
	load_program(program_filename);
	while(*program_filename++ != '\0');
    }
    CURRENT_LATCHES.Z = 1;
    CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
    memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);

    NEXT_LATCHES = CURRENT_LATCHES;

    RUN_BIT = TRUE;
}

/***************************************************************/
/*                                                             */
/* Procedure : main                                            */
/*                                                             */
/***************************************************************/
int main(int argc, char *argv[]) {                              
    FILE * dumpsim_file;

    /* Error Checking */
    if (argc < 3) {
	printf("Error: usage: %s <micro_code_file> <program_file_1> <program_file_2> ...\n",
	       argv[0]);
	exit(1);
    }

    printf("LC-3b Simulator\n\n");

    initialize(argv[1], argv[2], argc - 2);

    if ( (dumpsim_file = fopen( "dumpsim", "w" )) == NULL ) {
	printf("Error: Can't open dumpsim file\n");
	exit(-1);
    }

    while (1)
	get_command(dumpsim_file);

}

/***************************************************************/
/* Do not modify the above code.
   You are allowed to use the following global variables in your
   code. These are defined above.

   CONTROL_STORE
   MEMORY
   BUS

   CURRENT_LATCHES
   NEXT_LATCHES

   You may define your own local/global variables and functions.
   You may use the functions to get at the control bits defined
   above.

   Begin your code here 	  			       */
/***************************************************************/

/*
* Global Variables and Declarations
*/

int cycleCount; // keeps track of the total number of cycles 
int gateCS; // this is the gate control signal for MARMUX, etc.

/*
* Helper Functions
*/

int SEXT (int num, int signBit){
  // sign is the bit that we check for 
  int sign = num >> signBit; // aka bit shift 
  
  if(sign == 1){ // sign extend all 1 
    num = num << (31 - signBit);
    num = num >> (31 - signBit);
  } 

  return num;
}

void setCC(int num){
  if (num < 0) { // N bit on
    NEXT_LATCHES.N = 1;
    NEXT_LATCHES.Z = 0;
    NEXT_LATCHES.P = 0;
  } else if (num == 0) { // Z bit on 
    NEXT_LATCHES.N = 0;
    NEXT_LATCHES.Z = 1;
    NEXT_LATCHES.P = 0;
  } else if (num > 0){ // P bit on
    NEXT_LATCHES.N = 0;
    NEXT_LATCHES.Z = 0;
    NEXT_LATCHES.P = 1;
  }
}

/* 
* Evaluate the address of the next state according to the 
* micro sequencer logic. Latch the next microinstruction.
*/

void eval_micro_sequencer() {

    // J[5]
    // J[4]
    // J[3]  
    // J[2] = J[2] || (BEN && ~COND0 && COND1) i.e COND = 2
    // J[1] = J[1] || (R && COND0 && ~COND1) i.e COND = 1
    // J[0] = J[0] || (IR[11] && COND0 && COND1) i.e COND = 3
    // combine them  
    // MUX using IRD 
    // IRD = 0 -> 0,0,IR[15:12]
    // IRD = 1 -> J[5:0]

    // If the IRD control signal in the microinstruction corresponding to state 32 is 1,
    // the output MUX of the microsequencer will take its source from the six bits formed by 00 concatenated with the four opcode bits IR[15:12].

    if (GetIRD(CURRENT_LATCHES.MICROINSTRUCTION) == 1){ // this is corresponding to state 32

        int irN = (CURRENT_LATCHES.IR >> 11) & 0x1;
        int irZ = (CURRENT_LATCHES.IR >> 10) & 0x1;
        int irP = (CURRENT_LATCHES.IR >> 9) & 0x1;        

        NEXT_LATCHES.BEN = (irN & CURRENT_LATCHES.N) | (irZ & CURRENT_LATCHES.Z) | (irP & CURRENT_LATCHES.P);
        NEXT_LATCHES.STATE_NUMBER = (CURRENT_LATCHES.IR >> 12) && 0x0F; // source is stored in state number 

    } else {
        int tempJ = GetJ(CURRENT_LATCHES.MICROINSTRUCTION); // we need to seperate J, decompose that MF
        int Cond = GetCOND(CURRENT_LATCHES.MICROINSTRUCTION); // get COND
        // COND = 0, 1, 2, 3

        int J5 = (tempJ & 0x20) >> 5; // slide to the right
        int J4 = (tempJ & 0x10) >> 4; // slide to the left
        int J3 = (tempJ & 0x08) >> 3; // take it back now yall
        int J2 = (tempJ & 0x04) >> 2; // two hops this time
        int J1 = (tempJ & 0x02) >> 1; // criss cross!
        int J0 = tempJ & 0x00;
        int cond0;
        int cond1;


        // define condition bits based off our condition
        if (Cond == 3) { // COND = 11
            cond0 = 1;
            cond1 = 1;
        } else if (Cond == 2){ // COND = 10
            cond0 = 0;
            cond1 = 1;
        } else if (Cond = 1) { // COND = 01
            cond0 = 1;
            cond1 = 0;
        } else { // COND = 00
            cond0 = 0;
        cond1 = 0;
        }

        J2 = (J2 | (CURRENT_LATCHES.BEN & ~cond0 & cond1)) & 0x001; // OR it 
        J1 = (J1 | (CURRENT_LATCHES.READY & cond0 & ~cond1)) & 0x001;
        int IR11 = (CURRENT_LATCHES.IR >> 11) & 0x01; 
        J0 = (J0 | (IR11 & cond0 & cond1));

        NEXT_LATCHES.STATE_NUMBER = (J5 << 5) + (J4 << 4) + (J3 << 3) + (J2 << 2) + (J1 << 1) + J0; // Store that funky new number. Alrighty!
    }

    for (int i = 0; i < NEXT_LATCHES.MICROINSTRUCTION; i++){ //copy over the instruction from control store
        NEXT_LATCHES.MICROINSTRUCTION[i] = CONTROL_STORE[NEXT_LATCHES.STATE_NUMBER][i];
    }

}

/* 
* This function emulates memory and the WE logic. 
* Keep track of which cycle of MEMEN we are dealing with.  
* If fourth, we need to latch Ready bit at the end of 
* cycle to prepare microsequencer for the fifth cycle.  
*/
void cycle_memory() {
    // int cycleCount will keep track of the number of cycles for MEMEN
    if (GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION) == 0) { // if MIO is 0, we aren't writing/reading memory, no need
        return;
    } else { // otherwise, begin keeping track

        int memAddr = CURRENT_LATCHES.MAR >> 1; // rshf memory for offset (it's word aligned)
        int MAR0 = CURRENT_LATCHES.MAR & 0x1; // get if it's odd or even bit 
        cycleCount++; //increment memory 

        if (cycleCount == 4){
            NEXT_LATCHES.READY = 1; // fourth cycle -> enable ready bit
        }

        if(CURRENT_LATCHES.READY == 1) { // if the memory is ready -> aka 5th cycle
            if (GetR_W(CURRENT_LATCHES.MICROINSTRUCTION) == 1){ // oh hell yeah we writing
                int dataSize = GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION); // 0 - byte, 1 - word
                if (dataSize == 0){ // we are writing in a byte 
                    MEMORY[memAddr][MAR0] = (CURRENT_LATCHES.MDR & 0xFF); //write into the correct number
                } else { // we are writing in a word
                    MEMORY[memAddr][0] = (CURRENT_LATCHES.MDR & 0xFF); //write into the correct number
                    MEMORY[memAddr][1] = (CURRENT_LATCHES.MDR & 0xFF00) >> 8; //write into the correct number
                }
            }
            // Once memory is ready, reset
            cycleCount = 0;
            NEXT_LATCHES.READY = 0;
        } 
    }

}


/* 
* Datapath routine emulating operations before driving the bus.
* Evaluate the input of tristate drivers 
*        Gate_MARMUX,
*		 Gate_PC,
*		 Gate_ALU,
*		 Gate_SHF,
*		 Gate_MDR.
*/   
void eval_bus_drivers() {
    
    if (GetGATE_MARMUX(CURRENT_LATCHES.MICROINSTRUCTION)) {
        gateCS = 0x00;
        printf("Gate_MARMUX -> eval_bus_drivers");
    } 

    if (GetGATE_PC(CURRENT_LATCHES.MICROINSTRUCTION)) {
        gateCS = 0x02;
        printf("Gate_PC -> eval_bus_drivers");
    } 
    
    if (GetGATE_ALU(CURRENT_LATCHES.MICROINSTRUCTION)) {
        gateCS = 0x04;
        printf("Gate_ALU -> eval_bus_drivers");
    } 
    
    if (GetGATE_SHF(CURRENT_LATCHES.MICROINSTRUCTION)) {
        gateCS = 0x08;
        printf("Gate_SHF -> eval_bus_drivers");
    } 
    
    if (GetGATE_MDR(CURRENT_LATCHES.MICROINSTRUCTION)) {
        gateCS = 0x10;
        printf("Gate_MDR -> eval_bus_drivers");
    }
}



/* 
* Datapath routine for driving the bus from one of the 5 possible 
* tristate drivers. 
*/ 
void drive_bus() {
    
    if (gateCS == 0) {
        BUS = 0;
    } else if (gateCS == 0x1) { // gateMARMUX

        if (GATE_MARMUX == 0) {
            BUS = (CURRENT_LATCHES.IR & 0x0FF) << 1;
        } else { // GATE_MARMUX = 1 -> output adder
            int operand1;
            int operand2;

            // this is for the MARMUX selection -> ADDR1MUX and ADDR2MUX 

            if ((GetADDR1MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1)) { // ADDR1MUX - 1 = BaseR

                // ADDR1MUX is fed from SR1MUX and PC
                // SR1MUX - 1 = [11:9] - 2= [8:6]

                if (GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1) { // source 1 = 8:6
                    operand1 = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR >> 6) & 0x07];

                } else if (GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 0) { // source 2 = 11:9
                    operand1 = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR >> 9) & 0x07];
                } 
               
            } else { // ADDR1MUX selects PC if not BaseR 
                operand1 = CURRENT_LATCHES.PC;
            }

            // ADDR2MUX 
            // 0 - ZERO (value 0)
            // 1 - offset6 (SEXT[IR[5:0]])
            // 2 - PCoffset9 (SEXT[IR[8:0]])
            // 3 - PCoffset11 (SEXT[IR[10:0]])
            if (GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 0) { //selection is 0
                operand2 = 0;
            } else if (GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1) { //selection is offset6
                operand2 = SEXT((CURRENT_LATCHES.IR & 0x2F), 5);
            } else if (GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 2) { //selection is PCoffset9
                operand2 = SEXT((CURRENT_LATCHES.IR & 0x1FF), 8);
            } else if (GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 3) { //selection is PCoffset11
                operand2 = SEXT((CURRENT_LATCHES.IR & 0x7FF), 10);
            } else { // other selections?
                operand2 = 0;
            }

            if (GetLSHF1(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
                operand1 = operand1 >> 1; //left shift 1
            }
            
            BUS = Low16bits(operand1 + operand2);

        }

    } else if (gateCS == 0x2) { // GATE_PC is open
        BUS = CURRENT_LATCHES.PC;
    } else if (gateCS == 0x04) { // GATE_ALU is open

        // ALU is selected by ALUK (2 bits)
        // Fed into by SR2 MUX and SR1OUT
        // ALUK 
        // 0 - ADD, 1 - AND, 2 - XOR, 3 - PASSA

        int SR1;
        int SR2; // SR2 can either be offset or SR2MUX

        // SR1MUX - 1 = [11:9] - 2= [8:6]

        if (GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1) { // source 1 = 8:6
            SR1 = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR >> 6) & 0x07];
            // check if the add is with imm5 or with SR2 -> signed bit in IR[5]
        } else if (GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 0) { // source 2 = 11:9
           SR1 = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR >> 9) & 0x07];
        } 

        // check if the add is with imm5 or with SR2 -> signed bit in IR[5]
        // check steering bit
        if (((CURRENT_LATCHES.IR & 0x20) >> 5) == 1) { // 1 = imm5
            SR2 = SEXT(CURRENT_LATCHES.IR & 0x1F, 4); // imm5 is bits[4:0]
        } else { // 0 = SR2
            SR2 = CURRENT_LATCHES.REGS[CURRENT_LATCHES.IR & 0x07]; // SR2 is always located in bits[2:0]
        }
    
        // actually perform the ALU logic 
        if (GetALUK(CURRENT_LATCHES.MICROINSTRUCTION) == 0) { // selection is ADD     
            BUS = Low16bits(SR1 + SR2); // BUS = SR1 + SR2
        } else if (GetALUK(CURRENT_LATCHES.MICROINSTRUCTION) == 1) { // selection is AND
            BUS = Low16bits(SR1 & SR2); // BUS = SR1 & SR2
        } else if (GetALUK(CURRENT_LATCHES.MICROINSTRUCTION) == 2) { // selection is XOR
            BUS = Low16bits(SR1 ^ SR2); // BUS = SR1 ^ SR2
        } else if (GetALUK(CURRENT_LATCHES.MICROINSTRUCTION) == 3) { // selection is PASSA
            BUS = Low16bits(SR1); // pass A to the bus
        }

    } else if (gateCS == 0x08) { // GATE_SHF is open

        int SR1;

        if (GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1) { // source 1 = 8:6
            SR1 = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR >> 6) & 0x07];
            // check if the add is with imm5 or with SR2 -> signed bit in IR[5]
        } else if (GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 0) { // source 2 = 11:9
           SR1 = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR >> 9) & 0x07];
        } 


        int shiftAmount = CURRENT_LATCHES.IR & 0x2F; // IR[5:0]
        if (((CURRENT_LATCHES.IR & 0x10) >> 4) == 0) { // this is for LSHF
            BUS = SR1 << shiftAmount;
        } else if (((CURRENT_LATCHES.IR & 0x20) >> 5) == 0) { // this is for RSHFL
            BUS = SR1 >> shiftAmount;
        } else { // this is for RSHFA
            BUS = SEXT(SR1, 15) >> shiftAmount;
        }

    } else if (gateCS == 0x10) { // GATE_MDR is open 

        int dataSize = GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION);
        int MAR0 = CURRENT_LATCHES.MAR & 0x01; // MAR[0]

        if (dataSize = 0) { // if dataSize is 0 -> byte, if 1 -> word
            if (MAR0 == 1){
                BUS = Low16bits(SEXT(((CURRENT_LATCHES.MDR & 0xFF00) >> 8), 7)); // get the odd number
            } else {
                BUS = Low16bits(SEXT((CURRENT_LATCHES.MDR & 0xFF), 7)); // get the even number
            }
        } else { // PLUH!!!! WORD!!! SAY LESS!!! 
            BUS = Low16bits(CURRENT_LATCHES.MDR);
        }
        
    } 

}

/* 
   * Datapath routine for computing all functions that need to latch
   * values in the data path at the end of this cycle.  Some values
   * require sourcing the bus; therefore, this routine has to come 
   * after drive_bus.
   */
void latch_datapath_values() {

    if (GetLD_MAR(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
        NEXT_LATCHES.MAR = Low16bits(BUS);
    }

    if (GetLD_MDR(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
        if (GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
            int dataSize = GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION);
            int MAR0 = CURRENT_LATCHES.MAR & 0x1; 

            if (CURRENT_LATCHES.READY == 1){ // we now read from memory 
                if (dataSize == 0){ // read a byte
                    CURRENT_LATCHES.MDR = SEXT(MEMORY[CURRENT_LATCHES.MAR >> 1][MAR0], 7);
                } else { // read a word
                    CURRENT_LATCHES.MDR = ((MEMORY[CURRENT_LATCHES.MAR >> 1][1] & 0xFF)<< 8) + (MEMORY[CURRENT_LATCHES.MAR >> 1][0] & 0xFF);
                }
                
                NEXT_LATCHES.READY = 0;
                cycleCount = 0; //reset everything
            }
        } else {
            if (GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION) == 1) {
                NEXT_LATCHES.MDR = Low16bits(BUS);
            } else {
                NEXT_LATCHES.MDR = BUS & 0xFF;
            }
        }
    }

    if (GetLD_IR(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
        NEXT_LATCHES.IR = Low16bits(BUS);
    }

    if (GetLD_REG(CURRENT_LATCHES.MICROINSTRUCTION) == 1){

    }

    if (GetLD_CC(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
        setCC(SEXT(Low16bits(BUS), 15));
    }

    if (GetLD_PC(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
        int tempMux = GetPCMUX(CURRENT_LATCHES.MICROINSTRUCTION);
        // PCMUX holds 3 values 
        // 0 - PC + 2
        // 1 - BUS -> get value from BUS
        // 2 - ADDER -> address adder

        if (tempMux == 0) { // PC + 2
            NEXT_LATCHES.PC = CURRENT_LATCHES.PC + 2;
        } else if (tempMux == 1) { // BUS
            NEXT_LATCHES.PC = BUS;
        } else if (tempMux == 2) { // ADDER
            
            int operand1;
            int operand2;

            if ((GetADDR1MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1)) { // ADDR1MUX - 1 = BaseR

                // ADDR1MUX is fed from SR1MUX and PC
                // SR1MUX - 1 = [11:9] - 2= [8:6]

                if (GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1) { // source 1 = 8:6
                    operand1 = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR >> 6) & 0x07];

                } else if (GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 0) { // source 2 = 11:9
                    operand1 = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR >> 9) & 0x07];
                } 
               
            } else { // ADDR1MUX selects PC if not BaseR 
                operand1 = CURRENT_LATCHES.PC;
            }

            // ADDR2MUX 
            // 0 - ZERO (value 0)
            // 1 - offset6 (SEXT[IR[5:0]])
            // 2 - PCoffset9 (SEXT[IR[8:0]])
            // 3 - PCoffset11 (SEXT[IR[10:0]])
            if (GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 0) { //selection is 0
                operand2 = 0;
            } else if (GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1) { //selection is offset6
                operand2 = SEXT((CURRENT_LATCHES.IR & 0x2F), 5);
            } else if (GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 2) { //selection is PCoffset9
                operand2 = SEXT((CURRENT_LATCHES.IR & 0x1FF), 8);
            } else if (GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION) == 3) { //selection is PCoffset11
                operand2 = SEXT((CURRENT_LATCHES.IR & 0x7FF), 10);
            } else { // other selections?
                operand2 = 0;
            }

            if (GetLSHF1(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
                operand1 = operand1 >> 1; //left shift 1
            }

            NEXT_LATCHES.PC = operand1 + operand2;

        }
    }

}