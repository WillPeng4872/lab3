/*
    Name 1: Dhruv Bansal
    UTEID 1: db45988
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
    printf("STATE_NUMBER : 0x%.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);//remember to change later TODO please for the love of god
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

int print = 0; //remember to change rdump in shell code
int SEXT(int number, int number_bits){ // takes in an int, returns sign extended to 16 bits.

  int bit_check = number >> (number_bits - 1); // grab the MSB
  bit_check = bit_check & 0x01;                // check if zero or one
  if (bit_check)
  { // return correct format
    return ((number | (0xFFFF << number_bits)) & 0xFFFF);
  }
  return number;
}
int RSHF(int SR, int ammount4, int bit){
  int temp = CURRENT_LATCHES.REGS[SR];
  for (int i = 0; i < ammount4; i++)
  {
    temp = temp >> 1;
    temp = temp | (bit << 15);
  }
  return Low16bits(temp);
}

void eval_micro_sequencer() {

  /* 
   * Evaluate the address of the next state according to the 
   * micro sequencer logic. Latch the next microinstruction.
   */

    //printf("%d\n", GetIRD(CURRENT_LATCHES.MICROINSTRUCTION));
    //for (int i = 0; i<35; i++) printf("%x", (CURRENT_LATCHES.MICROINSTRUCTION[i]));
    //printf("\n");
    int next_state;
    if (GetIRD(CURRENT_LATCHES.MICROINSTRUCTION)){ //means next state is based off of opcode
        next_state = (CURRENT_LATCHES.IR & 0xF000) >> 12;

    }
    else{ //else, next state based on state sequencer
        next_state = (GetJ(CURRENT_LATCHES.MICROINSTRUCTION));
        int j2 = ((next_state & 0x4)>>2) | ((CURRENT_LATCHES.BEN && !(GetCOND(CURRENT_LATCHES.MICROINSTRUCTION) & 0x1)) && ((GetCOND(CURRENT_LATCHES.MICROINSTRUCTION) & 0x2)>>1));
        int j1 = ((next_state & 0x2)>>1) | ((CURRENT_LATCHES.READY && (GetCOND(CURRENT_LATCHES.MICROINSTRUCTION) & 0x1)) && (!((GetCOND(CURRENT_LATCHES.MICROINSTRUCTION) & 0x2)>>1)));
        int j0 = (next_state & 0x1) | ((((CURRENT_LATCHES.IR & 0x800) >> 11) && (GetCOND(CURRENT_LATCHES.MICROINSTRUCTION) & 0x1)) && ((GetCOND(CURRENT_LATCHES.MICROINSTRUCTION) & 0x2)>>1));
        next_state = (next_state & 0x20) + (next_state & 0x10) +  (next_state & 0x08) + (j2 << 2) + (j1 << 1) + j0;
    }
    NEXT_LATCHES.STATE_NUMBER = next_state;
    memcpy(NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[next_state], sizeof(int)*CONTROL_STORE_BITS);

}

int memory_out;
int mem_cycle;
void read_word(){
    mem_cycle++;
    if (print) printf("mem_cycles: %d\n", mem_cycle);
    
    if(MEM_CYCLES == mem_cycle){
        memory_out = (MEMORY[CURRENT_LATCHES.MAR >> 1][1] <<8) | (MEMORY[CURRENT_LATCHES.MAR >> 1][0]);
        if (print) printf("memory read: %.4x from location: %.4x\n", memory_out, CURRENT_LATCHES.MAR);
        NEXT_LATCHES.READY = 1;
        mem_cycle = 0;
    }
}
void store_byte(){
    mem_cycle++;
    
    if(MEM_CYCLES == mem_cycle){
        (CURRENT_LATCHES.MAR & 0x01)? (MEMORY[CURRENT_LATCHES.MAR >> 1][1] = (CURRENT_LATCHES.MDR & 0xFF)) : (MEMORY[CURRENT_LATCHES.MAR >> 1][0] = (CURRENT_LATCHES.MDR & 0xFF));
        NEXT_LATCHES.READY = 1;
        mem_cycle = 0;
    }
}
void store_word(){
    mem_cycle++;
    
    if(MEM_CYCLES == mem_cycle){
        MEMORY[CURRENT_LATCHES.MAR >> 1][1] = (CURRENT_LATCHES.MDR>> 8) & 0xFF;
        MEMORY[CURRENT_LATCHES.MAR >> 1][0] = CURRENT_LATCHES.MDR & 0xFF;
        NEXT_LATCHES.READY = 1;
        mem_cycle = 0;
    }
}
void cycle_memory() {
 
  /* 
   * This function emulates memory and the WE logic. 
   * Keep track of which cycle of MEMEN we are dealing with.  
   * If fourth, we need to latch Ready bit at the end of 
   * cycle to prepare microsequencer for the fifth cycle.  
   */
    int mem_state = GetJ(CURRENT_LATCHES.MICROINSTRUCTION);
    NEXT_LATCHES.READY = 0;
    switch (mem_state){
        case 16: store_word(); break;
        case 17: store_byte(); break;
        case 25: read_word(); break;
        case 28: read_word(); break;
        case 29: read_word(); break;
        case 33: read_word(); break;
        default: mem_cycle = 0; break; 
    }

}


int MarMuxVal, ALUVal, SHFVal, MDRVal, PCVal;
void eval_bus_drivers() {

  /* 
   * Datapath routine emulating operations before driving the bus.
   * Evaluate the input of tristate drivers 
   *             Gate_MARMUX,
   *		 Gate_PC,
   *		 Gate_ALU,
   *		 Gate_SHF,
   *		 Gate_MDR.
   */
    BUS = 0;    
    int Addr2MuxVal = 0;
    int Addr2MuxState = GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION);
    if (print) printf("ADDR2MUX is: %d\n", Addr2MuxState);
    switch (Addr2MuxState){
        case 0: Addr2MuxVal = 0;break;
        case 1: Addr2MuxVal = Low16bits(SEXT(CURRENT_LATCHES.IR & 0x3F, 6)); break;
        case 2: Addr2MuxVal = Low16bits(SEXT(CURRENT_LATCHES.IR & 0x1FF, 9)); break;
        case 3: Addr2MuxVal = Low16bits(SEXT(CURRENT_LATCHES.IR & 0x7FF, 11)); break;
    }
    if (print) printf("LSHF1: %d\n", GetLSHF1(CURRENT_LATCHES.MICROINSTRUCTION));
    if (GetLSHF1(CURRENT_LATCHES.MICROINSTRUCTION)){
        Addr2MuxVal = Addr2MuxVal << 1;
    }
    if (print) printf("ADDR2MUX Val is: %d\n", Addr2MuxVal);
    int Addr1MuxVal = 0;
    int Addr1MuxState = GetADDR1MUX(CURRENT_LATCHES.MICROINSTRUCTION);
    int BaseR = (CURRENT_LATCHES.IR >> 6) & 0x7; 
    switch (Addr1MuxState){
        case 0: Addr1MuxVal = CURRENT_LATCHES.PC; break;
        case 1: Addr1MuxVal = CURRENT_LATCHES.REGS[(BaseR)]; break;
    }
    if (print) printf("ADDR1MUX is: %d\n", Addr1MuxState);
    if (print) printf("ADDR1MUX Val is: %.4x\n", Addr1MuxVal);
    int adder = Addr2MuxVal + Addr1MuxVal;
    if (GetGATE_MARMUX(CURRENT_LATCHES.MICROINSTRUCTION)){
        if (GetMARMUX(CURRENT_LATCHES.MICROINSTRUCTION)){
            MarMuxVal = adder;
        }else{
            MarMuxVal = SEXT(CURRENT_LATCHES.IR & 0xFF, 8) << 1;
        }
    }
    if (GetGATE_ALU(CURRENT_LATCHES.MICROINSTRUCTION)){
        int AlukState = GetALUK(CURRENT_LATCHES.MICROINSTRUCTION);
        int SR1 = GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION)? ((CURRENT_LATCHES.IR >> 6) & 0x7 ): ((CURRENT_LATCHES.IR >> 9) & 0x7);
        int SR2 = (CURRENT_LATCHES.IR) & 0x7;
        int mode = (CURRENT_LATCHES.IR >> 5) & 0x1;
        switch (AlukState){
            case 0: ALUVal = CURRENT_LATCHES.REGS[SR1];
                if(mode){ ALUVal += SEXT(CURRENT_LATCHES.IR & 0x1F,5);}
                else{ ALUVal += CURRENT_LATCHES.REGS[SR2];}
                break;
            case 1: ALUVal = CURRENT_LATCHES.REGS[SR1];
                if(mode){ ALUVal = ALUVal & SEXT(CURRENT_LATCHES.IR & 0x1F,5);}
                else{ ALUVal = ALUVal & CURRENT_LATCHES.REGS[SR2];}
                break;
            case 2: ALUVal = CURRENT_LATCHES.REGS[SR1];
                if(mode){ ALUVal = ALUVal ^ SEXT(CURRENT_LATCHES.IR & 0x1F,5);}
                else{ ALUVal = ALUVal ^ CURRENT_LATCHES.REGS[SR2];}
                break;
            case 3: ALUVal = CURRENT_LATCHES.REGS[SR1]; break;
        }
    }
    if (GetGATE_SHF(CURRENT_LATCHES.MICROINSTRUCTION)){
        int SR1 = GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION)? ((CURRENT_LATCHES.IR >> 6) & 0x7 ): ((CURRENT_LATCHES.IR >> 9) & 0x7);
        int ammount4 = (CURRENT_LATCHES.IR & 0xF);
        int right_left = (CURRENT_LATCHES.IR >> 4) & 0x1;
        int logical = (CURRENT_LATCHES.IR >> 5) & 0x1;
        if (print) printf("The SR is: %.4x\n", SR1);
        if (print) printf("The SRVal is: %.4x\n", CURRENT_LATCHES.REGS[SR1]);
        if (right_left){
            if (logical){
                int bit = (CURRENT_LATCHES.REGS[SR1] >>15) & 0x1;
                SHFVal = RSHF(SR1, ammount4, bit);
            }
            else{
                SHFVal = RSHF(SR1, ammount4, 0);
            }
        }
        else{
            SHFVal = (CURRENT_LATCHES.REGS[SR1] << ammount4);
        } 
        if (print) printf("The SHFVal is: %.4x\n", SHFVal);
    }
    if (GetGATE_MDR(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION)){
            MDRVal = CURRENT_LATCHES.MDR;
        }else{
            if(CURRENT_LATCHES.MAR & 0x01){
                MDRVal = SEXT((CURRENT_LATCHES.MDR>>8) & 0xFF, 8);
            }else{
                MDRVal = SEXT(CURRENT_LATCHES.MDR & 0xFF, 8);
            }
        }
    }
    if (GetLD_PC(CURRENT_LATCHES.MICROINSTRUCTION)){
        int PCMUXVal = GetPCMUX(CURRENT_LATCHES.MICROINSTRUCTION);
        switch (PCMUXVal){
            case 0: PCVal = CURRENT_LATCHES.PC +2; break;
            case 1: PCVal = BUS; break;
            case 2: PCVal = adder; break;
        }
    }

}


void drive_bus() {

  /* 
   * Datapath routine for driving the bus from one of the 5 possible 
   * tristate drivers. 
   */
   
    if (GetGATE_ALU(CURRENT_LATCHES.MICROINSTRUCTION)){
        BUS = ALUVal;
    } else if (GetGATE_SHF(CURRENT_LATCHES.MICROINSTRUCTION)){
        BUS = SHFVal;
    } else if (GetGATE_PC(CURRENT_LATCHES.MICROINSTRUCTION)){
        BUS = CURRENT_LATCHES.PC;
    } else if (GetGATE_MDR(CURRENT_LATCHES.MICROINSTRUCTION)){
        BUS = MDRVal;
    } else if (GetGATE_MARMUX(CURRENT_LATCHES.MICROINSTRUCTION)){
        BUS = MarMuxVal;
    }
}


void latch_datapath_values() {

  /* 
   * Datapath routine for computing all functions that need to latch
   * values in the data path at the end of this cycle.  Some values
   * require sourcing the bus; therefore, this routine has to come 
   * after drive_bus.
   */
   //check ld for PC, CC, IR, MDR, MAR, REG, BEN
    if(GetLD_PC(CURRENT_LATCHES.MICROINSTRUCTION)){
        NEXT_LATCHES.PC = Low16bits(PCVal);
    }       
    if(GetLD_IR(CURRENT_LATCHES.MICROINSTRUCTION)){
        NEXT_LATCHES.IR = Low16bits(BUS);
    }
    if(GetLD_CC(CURRENT_LATCHES.MICROINSTRUCTION)){
        int temp = Low16bits(BUS);
        if (temp ==0){
            NEXT_LATCHES.Z = 1;
            NEXT_LATCHES.P = 0;
            NEXT_LATCHES.N = 0;
        } else if (temp >> 15){
            NEXT_LATCHES.Z = 0;
            NEXT_LATCHES.P = 0;
            NEXT_LATCHES.N = 1;
        }else{
            NEXT_LATCHES.Z = 0;
            NEXT_LATCHES.P = 1;
            NEXT_LATCHES.N = 0;
        }
    }
    if(GetLD_MDR(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION)){
            NEXT_LATCHES.MDR = Low16bits(memory_out);
        }else{
            if(GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION)){
                NEXT_LATCHES.MDR = Low16bits(BUS);
            }else{
                NEXT_LATCHES.MDR = Low16bits(BUS & 0xFF);
            }
        }
    }
    if(GetLD_MAR(CURRENT_LATCHES.MICROINSTRUCTION)){
        NEXT_LATCHES.MAR = Low16bits(BUS);
    }
    if(GetLD_REG(CURRENT_LATCHES.MICROINSTRUCTION)){
        int dr = (CURRENT_LATCHES.IR>>9) & 0x7;
        if(GetDRMUX(CURRENT_LATCHES.MICROINSTRUCTION)){
            dr = 7;
        }
        NEXT_LATCHES.REGS[dr] = Low16bits(BUS);
    }
    if(GetLD_BEN(CURRENT_LATCHES.MICROINSTRUCTION)){
        if (GetLD_BEN(CURRENT_LATCHES.MICROINSTRUCTION)){
            NEXT_LATCHES.BEN = (NEXT_LATCHES.N & ((CURRENT_LATCHES.IR >>11) & 0x01)) || (NEXT_LATCHES.Z & ((CURRENT_LATCHES.IR >>10) & 0x01)) || (NEXT_LATCHES.P & ((CURRENT_LATCHES.IR >>9) & 0x01));
        }
    }
}
//list of state diagrams to test
/*
add //tested
and //tested
xor //tested
trap //tested
shf //tested
lea //tested
ldb //tested
ldw //tested
stw //tested
stb //tested
jsr //tested
jsrr //tested
jmp //tested
br //tested
*/