// Author: Jason Tennyson
// Date: 7-10-11
// File: main.c
//
// This is the design for the parent module of Jason Tennyson's Thesis.
// This design is made for a PSoC CY8C28433-24PVXI.
//
// Controller Packet Structure (each field is a byte)
// -----------------------------------------------------
// All Packets:
// START BYTE/START BYTE/SOURCE ID/DESTINATION ID/COMMAND TYPE/PARAM 1/.../PARAM N/END TRANSMIT
//
// Servo Packet Structure (each field is a byte)
// -----------------------------------------------------
// Source Packets:
// START BYTE/START BYTE/DESTINATION ID/LENGTH/COMMAND TYPE/PARAM 1/.../PARAM N/CHECKSUM
//
// Return Packets:
// START BYTE/START BYTE/SOURCE ID/LENGTH/ERROR/PARAM1/.../PARAM N/CHECKSUM

#include <m8c.h>        	// Part-specific constants and macros.
#include "PSoCAPI.h"    	// PSoC API definitions for all User Modules.
#include "psocdynamic.h"	// Required for dynamically swapping configurations at run time.
#include <stdlib.h>			// Required for converting character arrays to and from floats and ints.

//#include <string.h>

// These are declarations of all of the timer interrupts that are used for all configurations.
#pragma interrupt_handler TX_TIMEOUT_ISR
#pragma interrupt_handler RX_TIMEOUT_ISR

// These defines are used as parameters of the configToggle function.
// Passing one or the other in the function call switches the system between PC and RX modes.
#define		PC_MODE						(1)
#define		RX_MODE						(2)

// These defines are used as comparisons to find what port the newest module is connected to.
#define		PORT_1						('1')
#define		PORT_2						('2')
#define		PORT_3						('3')
#define		PORT_4						('4')

// This is the module type identifier.
#define		TYPE						('2')

// These defines are used as transmission indicators.
#define		START_TRANSMIT				(252)	// Indicates the beginning of a transmission.
#define		END_TRANSMIT				(253)	// Indicates the end of a transmission.
#define		COMMAND_TYPE_SPACE			(200)	// The number where reserved command types start.
#define		HELLO_BYTE					(200)	// Indicates parent is ready to talk.
#define		ID_ASSIGNMENT				(201)	// Indicates an ID assignment from the parent.
#define		ID_ASSIGN_OK				(202)	// Indicates an ID assignment is acknowledged.
#define		PING						(203)	// Indicates that someone is pinging someone else.
#define		CLEAR_CONFIG				(204)	// Indicates that the parent is asking for a config clear.
#define		CONFIG_CLEARED				(205)	// Indicates that a module has cleared its own config.
#define		PARENT_ID					(0)		// The parent node's ID.
#define		BROADCAST					(254)	// The broadcast ID for talking to all nodes.
#define		BLANK_MODULE_ID				(251)	// This is the ID of an unconfigured module.
#define		SERVO_START					(255)	// The start byte of a servo.

// These defines are used to fill in the instruction we are using on the servo.
#define		PING_SERVO					(1)		// This is the instruction number for ping.
#define		READ_SERVO					(2)		// This is the instruction number for a read.
#define		WRITE_SERVO					(3)		// This is the instruction number for a write.
#define		RESET_SERVO					(6)		// This is the instruction to reset the servo EEPROM.

// These defines are used for transmission timing.
#define 	RX_TIMEOUT_DURATION			(5)		// This is receive wait time in 1 ms units.

// These defines are used for the initial probing stage.
#define		INIT_WAIT_TIME				(50)	// Initial wait time between module probes.
#define		MAX_TIMEOUTS				(50)	// Number of timeouts allowed before hello mode exit.

// This is the maximum number of allowable modules per branch out from the parent.
#define		MAX_MODULES					(250)

// Receives a mode identifier and toggles to that mode.
void configToggle(int mode);
// Pings the index passed to it. Returns 1 on success, 0 on fail.
int pingModule(int module_id);
// Assigns an ID to a module.
int assignID(int assigned_ID);
// Attempts to read a valid transmission and store it.
int validTransmission(void);
// Reads a PC command and translates it to the correct packet type.
void decodeTransmission(void);
// Sends out a hello message packet.
void sayHello(void);
// Servo instruction function that sends read or write commands.
void servoInstruction(char id, char length, char instruction, char address, char value);
// Servo instruction function that sends long two-byte write commands.
void longServoInstruction(char id, char length, char instruction, char address, char value1, char value2);
// Immediately performs a non-blocking read char operation, and returns 0 upon failure.
char iReadChar(void);
// Performs a blocking read char operation.
char readChar(void);
// Checks the current mode and unloads the configuration for that mode.
void unloadAllConfigs(void);
// Unloads the configuration corresponding to the number passed to it.
void unloadConfig(int config_num);
// Initialization function for the child module controllers.
void initializeChildren(void);
// Static wait time of approximately 50 microseconds for use after starting a transmission.
void xmitWait(void);
// Listen for a child and record the port value.
void childListen(void);

int TIMEOUT;				// This flag is incremented if there is a timeout.
int NUM_MODULES;			// Stores the number of modules that have been discovered.
int STATE;					// Stores the current configuration state of the system.
char CHILD;					// The child port value stored from initialization.

char COMMAND_SOURCE;		// Stores who the current command is from.
char COMMAND_DESTINATION;	// Stores who the current command is for.
char COMMAND_TYPE;			// Stores the type of command that was just read.
char PARAM[10];				// Stores a parameters that accompanies the command (if any).

void main()
{	
	NUM_MODULES = 0;	// Initialize the number of modules.
	STATE = 0;			// Initialize the current hardware state.
	
	// Activate GPIO ISR.
	M8C_EnableIntMask(INT_MSK0,INT_MSK0_GPIO);
	
	// Turn on global interrupts for the transmission timeout timer.
	M8C_EnableGInt;
	
	while(1)
	{
		// If there are no modules, find some. Otherwise, look for computer commands.
		if(!NUM_MODULES)
		{
			initializeChildren();
		}
		else if(COMP_SERIAL_bCmdCheck())
		{
			decodeTransmission();
		}
	}
}

int pingModule(int module_id)
{
	// Toggle into PC mode.
	configToggle(PC_MODE);
	
	// Transmit a ping to everyone.
	TX_REPEATER_14_PutChar(START_TRANSMIT);	// Start byte one
	TX_REPEATER_23_PutChar(START_TRANSMIT);		// Start byte one
	TX_REPEATER_14_PutChar(START_TRANSMIT);	// Start byte two
	TX_REPEATER_23_PutChar(START_TRANSMIT);		// Start byte two
	TX_REPEATER_14_PutChar(PARENT_ID);			// My ID
	TX_REPEATER_23_PutChar(PARENT_ID);			// My ID
	TX_REPEATER_14_PutChar(module_id);			// Destination ID
	TX_REPEATER_23_PutChar(module_id);			// Destination ID
	TX_REPEATER_14_PutChar(PING);				// This is a ping response
	TX_REPEATER_23_PutChar(PING);				// This is a ping response
	TX_REPEATER_14_PutChar(END_TRANSMIT);		// This is the end of this transmission
	TX_REPEATER_23_PutChar(END_TRANSMIT);		// This is the end of this transmission
	TX_REPEATER_14_PutChar(END_TRANSMIT);		// This is the end of this transmission
	TX_REPEATER_23_PutChar(END_TRANSMIT);		// This is the end of this transmission
	
	// Wait for the transmission to finish.
	while(!(TX_REPEATER_14_bReadTxStatus() & TX_REPEATER_14_TX_COMPLETE));
	while(!(TX_REPEATER_23_bReadTxStatus() & TX_REPEATER_23_TX_COMPLETE));
	
	// Make completely sure we're done.
	xmitWait();
	
	// Switch to listening mode.
	configToggle(RX_MODE);
	
	// Listen for the response.
	while(TIMEOUT < RX_TIMEOUT_DURATION)
	{
		if(validTransmission())
		{
			// If the response is what we are looking for.
			if(COMMAND_TYPE == PING)
			{
				// If this is for me, check who it was from.
				if(COMMAND_DESTINATION == PARENT_ID)
				{
					// If it's from the right module, return 1.
					if(COMMAND_SOURCE == module_id)
					{
						return 1;
					}
				}
			}
		}
	}

	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	
	return 0;
}

int assignID(int assigned_ID)
{	
	// Switch to PC mode.
	configToggle(PC_MODE);

	// Transmit an ID assignment.
	TX_REPEATER_14_PutChar(START_TRANSMIT);	// Start byte one
	TX_REPEATER_23_PutChar(START_TRANSMIT);		// Start byte one
	TX_REPEATER_14_PutChar(START_TRANSMIT);	// Start byte two
	TX_REPEATER_23_PutChar(START_TRANSMIT);		// Start byte two
	TX_REPEATER_14_PutChar(PARENT_ID);			// My ID
	TX_REPEATER_23_PutChar(PARENT_ID);			// My ID
	TX_REPEATER_14_PutChar(BLANK_MODULE_ID);	// Destination ID
	TX_REPEATER_23_PutChar(BLANK_MODULE_ID);	// Destination ID
	TX_REPEATER_14_PutChar(ID_ASSIGNMENT);		// This is an ID assignment
	TX_REPEATER_23_PutChar(ID_ASSIGNMENT);		// This is an ID assignment
	TX_REPEATER_14_PutChar(assigned_ID);		// This is the new ID
	TX_REPEATER_23_PutChar(assigned_ID);		// This is the new ID
	TX_REPEATER_14_PutChar(END_TRANSMIT);		// This is the end of this transmission
	TX_REPEATER_23_PutChar(END_TRANSMIT);		// This is the end of this transmission
	TX_REPEATER_14_PutChar(END_TRANSMIT);		// This is the end of this transmission
	TX_REPEATER_23_PutChar(END_TRANSMIT);		// This is the end of this transmission
	
	// Wait for the transmission to finish.
	while(!(TX_REPEATER_14_bReadTxStatus() & TX_REPEATER_14_TX_COMPLETE));
	while(!(TX_REPEATER_23_bReadTxStatus() & TX_REPEATER_23_TX_COMPLETE));
	
	// Make completely sure we're done.
	xmitWait();
	
	// Switch to listening mode.
	configToggle(RX_MODE);
	
	// Listen for the response.
	while(TIMEOUT < RX_TIMEOUT_DURATION)
	{
		if(validTransmission())
		{
			// If this is the response we are looking for.
			if(COMMAND_TYPE == ID_ASSIGN_OK)
			{
				// If this is for me, check who it was from.
				if(COMMAND_DESTINATION == PARENT_ID)
				{
					// If it is from the right module, return 1.
					if(COMMAND_SOURCE == assigned_ID)
					{
						return 1;
					}
				}
			}
		}
	}
	
	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	
	return 0;
}

// This function transmits a hello message.
void sayHello(void)
{
	// Toggle into PC mode.
	configToggle(PC_MODE);
	
	// Transmit an ID assignment.
	TX_REPEATER_14_PutChar(START_TRANSMIT);	// Start byte one
	TX_REPEATER_23_PutChar(START_TRANSMIT);		// Start byte one
	TX_REPEATER_14_PutChar(START_TRANSMIT);	// Start byte two
	TX_REPEATER_23_PutChar(START_TRANSMIT);		// Start byte two
	TX_REPEATER_14_PutChar(PARENT_ID);			// My ID
	TX_REPEATER_23_PutChar(PARENT_ID);			// My ID
	TX_REPEATER_14_PutChar(BLANK_MODULE_ID);	// Destination ID
	TX_REPEATER_23_PutChar(BLANK_MODULE_ID);	// Destination ID
	TX_REPEATER_14_PutChar(HELLO_BYTE);		// This is a hello message
	TX_REPEATER_23_PutChar(HELLO_BYTE);			// This is a hello message
	TX_REPEATER_14_PutChar(END_TRANSMIT);		// This is the end of this transmission
	TX_REPEATER_23_PutChar(END_TRANSMIT);		// This is the end of this transmission
	TX_REPEATER_14_PutChar(END_TRANSMIT);		// This is the end of this transmission
	TX_REPEATER_23_PutChar(END_TRANSMIT);		// This is the end of this transmission
	
	// Wait for the transmission to finish.
	while(!(TX_REPEATER_14_bReadTxStatus() & TX_REPEATER_14_TX_COMPLETE));
	while(!(TX_REPEATER_23_bReadTxStatus() & TX_REPEATER_23_TX_COMPLETE));
	
	// Make completely sure we're done.
	xmitWait();
	
	// Switch back to listening mode.
	configToggle(RX_MODE);
}

// This function returns whether or not a valid transmission has been received.
int validTransmission(void)
{
	int i = 0;			// Index for looping.
	char tempByte = 0;	// Temporary byte storage.
	
	// These loops and conditionals are arranged in a way that allows this read
	// operation to be completely non-blocking.
	while(TIMEOUT < RX_TIMEOUT_DURATION)
	{
		// Wait until we read a start transmit byte.
		if(iReadChar() == START_TRANSMIT)
		{
			// While we haven't timed out, look for something other than a start byte.
			while(TIMEOUT < RX_TIMEOUT_DURATION)
			{
				// If we find a nonzero byte...
				if(tempByte = iReadChar())
				{
					// If the byte we found isn't a start byte...
					if(tempByte != START_TRANSMIT)
					{
						// This byte is probably the command source.
						COMMAND_SOURCE = tempByte;
						
						// Look for the rest of the command before we time out.
						while(TIMEOUT < RX_TIMEOUT_DURATION)
						{
							// If we read another nonzero byte...
							if(tempByte = iReadChar())
							{
								// If that byte is in the command type indicator space...
								if(tempByte >= COMMAND_TYPE_SPACE)
								{
									// Store the command type.
									COMMAND_TYPE = tempByte;
									
									// Continue reading if we have not timed out yet.
									while(TIMEOUT < RX_TIMEOUT_DURATION)
									{
										// If we read a nonzero byte...
										if(tempByte = iReadChar())
										{
											// Store the parameter if it is not the end indicator.
											if(tempByte != END_TRANSMIT)
											{
												PARAM[i] = tempByte;
												i++;
											}
											else
											{
												return 1;
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	
	return 0;
}

// This function decodes the transmission and takes the correct action.
void decodeTransmission(void)
{
	char* param;			// Stores the most recent parameter from the buffer.
	char ID = 0;			// Stores the target module ID.
	char tempByte = 0;		// Temporary byte storage.
	char angle[2];			// Store the two angle bytes for the servo.
	char speed[2];			// Store the two speed bytes for the servo.
	int total = 0;			// Used to store the converted total of angle or speed bytes.
	int runningTotal = 0;	// Used as part of the dynamic checksum calculation.
	
	// Read a parameter from the buffer.
	if(param = COMP_SERIAL_szGetParam())
	{
		if((param[0] == 'x') || (param[0] == 'X'))
		{
			// Reset the robot.
			NUM_MODULES = 0;
		}
		else if((param[0] == 'n') || (param[0] == 'N'))
		{
			itoa(param,NUM_MODULES,10);		// Convert the NUM_MODULES int to a char array.
			COMP_SERIAL_PutString(param);	// Send that array out to the PC.
			COMP_SERIAL_PutChar('\n');		// End the transmission with the PC.
		}
		else if((param[0] == 'w') || (param[0] == 'W'))
		{
			if(param = COMP_SERIAL_szGetParam())
			{
				// Convert the ID parameter to a char byte.
				ID = atoi(param);
				
				if(param = COMP_SERIAL_szGetParam())
				{
					if((param[0] == 'a') || (param[0] == 'A'))
					{
						if(param = COMP_SERIAL_szGetParam())
						{
							// Get the angle parameter and convert it to an integer.
							total = atoi(param);
							
							// Convert the integer into bytes.
							angle[0] = total%256;
							angle[1] = total/256;
							
							// Send the servo the angle.
							longServoInstruction(ID,5,WRITE_SERVO,30,angle[0],angle[1]);
						}
					}
					else if((param[0] == 'p') || (param[0] == 'P'))
					{
						if(param = COMP_SERIAL_szGetParam())
						{
							// Send the servo the desired power value.
							servoInstruction(ID,4,WRITE_SERVO,24,atoi(param));
						}
					}
					else if((param[0] == 's') || (param[0] == 'S'))
					{
						if(param = COMP_SERIAL_szGetParam())
						{
							// Get the speed parameter and convert it to an integer.
							total = atoi(param);
							
							// If no total, do nothing because 0 is no speed control (undesired).
							if(total)
							{
								// Convert the integer into bytes.
								speed[0] = total%256;
								speed[1] = total/256;
								
								// Write the speed value to the servo.
								longServoInstruction(ID,5,WRITE_SERVO,32,speed[0],speed[1]);
							}
						}
					}
				}
			}
		}
		else if((param[0] == 'r') || (param[0] == 'R'))
		{			
			if(param = COMP_SERIAL_szGetParam())
			{
				// Extract the target ID param and convert it to an integer.
				ID = atoi(param);
				
				if(param = COMP_SERIAL_szGetParam())
				{
					if((param[0] == 'a') || (param[0] == 'A'))
					{
						// Initialize the angle bytes to 0.
						angle[0] = 0;
						angle[1] = 0;
						
						// Send a request to the servo for its angle.
						servoInstruction(ID,4,READ_SERVO,36,2);
						
						// Switch to read the response.
						configToggle(RX_MODE);
							
						// Loop until we read a response or time out.
						while(TIMEOUT < RX_TIMEOUT_DURATION)
						{
							// If the response is from the right ID...
							if(iReadChar() == ID)
							{
								while(TIMEOUT < RX_TIMEOUT_DURATION)
								{
									// The length of the response remainder should be 4.
									if(iReadChar() == 4)
									{
										// The error value should be 0 if successful.
										if(readChar() == 0)
										{
											// Grab the bytes from the buffer.
											angle[0] = readChar();
											angle[1] = readChar();
											
											// Switch to PC mode to forward the response.
											configToggle(PC_MODE);
											
											// Convert the bytes to an integer.
											total = ((angle[1])*256) + angle[0];
											
											// Convert the integer to a character array.
											itoa(param,total,10);
											
											// Write the response to the computer.
											COMP_SERIAL_PutString(param);
											COMP_SERIAL_PutChar('\n');

											// Force a timeout to exit all loops.
											TIMEOUT = RX_TIMEOUT_DURATION;
										}
										else
										{
											// Force a timeout to exit all loops.
											TIMEOUT = RX_TIMEOUT_DURATION;
										}
									}
								}
							}
						}
					}
					else if ((param[0] == 'p') || (param[0] == 'P'))
					{
						// Send a request to the servo for its power status.
						servoInstruction(ID,4,READ_SERVO,24,1);
						
						// Switch to read the response.
						configToggle(RX_MODE);
						
						// Loop until we read a response or time out.
						while(TIMEOUT < RX_TIMEOUT_DURATION)
						{
							if(iReadChar() == ID)
							{
								runningTotal = ID;
								// Loop until we read a response or time out.
								while(TIMEOUT < RX_TIMEOUT_DURATION)
								{
									// Check the length of the packet.
									if(iReadChar() == 3)
									{
										// Tack the value onto our running total.
										runningTotal += 3;
										
										// Loop until we read a response or time out.
										while(TIMEOUT < RX_TIMEOUT_DURATION)
										{
											// Check for the checksum or 1.
											if(tempByte = iReadChar())
											{
												// Switch to PC mode to forward the result.
												configToggle(PC_MODE);
												
												if((runningTotal%256) == (255-tempByte))
												{
													// Send a 0 if we hit the checksum.
													COMP_SERIAL_PutChar('0');
													COMP_SERIAL_PutChar('\n');
												}
												else
												{
													// Send a 1 if we hit it first.
													COMP_SERIAL_PutChar('1');
													COMP_SERIAL_PutChar('\n');
												}
		
												TIMEOUT = RX_TIMEOUT_DURATION;
											}
										}
									}
								}
							}
						}
					}
					else if ((param[0] == 't') || (param[0] == 'T'))
					{
						// If this isn't for the parent, ping the module to get a
						// status packet and return the data.
						if(ID == 0)
						{
							COMP_SERIAL_PutChar(TYPE);
							COMP_SERIAL_PutChar('\n');
						}
						else if(pingModule(ID))
						{
							configToggle(PC_MODE);
												
							COMP_SERIAL_PutChar(PARAM[0]);
							COMP_SERIAL_PutChar('\n');
						}
					}
					else if ((param[0] == 'c') || (param[0] == 'C'))
					{
						// If this isn't for the parent, ping the module to get a
						// status packet and return the data.
						if(ID == 0)
						{
							COMP_SERIAL_PutChar(CHILD);
							COMP_SERIAL_PutChar('\n');
						}
						else if(pingModule(ID))
						{	
							configToggle(PC_MODE);
							
							COMP_SERIAL_PutChar(PARAM[1]);
							COMP_SERIAL_PutChar('\n');
						}
					}
				}
			}
		}
	}
	
	// Reset the timeout and switch to PC mode.
	if(STATE != PC_MODE)
	{
		configToggle(PC_MODE);
	}
	else
	{
		TIMEOUT = 0;
		COMP_SERIAL_CmdReset();
	}
}

// This function receives a destination, command length, instruction type, address, and value.
// With these parameters, the function sends a packet to the communication bus.
void servoInstruction(char id, char length, char instruction, char address, char value)
{
	char checksum;	// The checksum byte value.
	int total;		// The total for use in calculating the checksum.
	
	// Get the total of all bytes.
	total = id + length + instruction + address + value;
	
	// Calculate the checksum value for our servo communication.
	checksum = 255-(total%256);
	
	// Talk to the servo.
	TX_REPEATER_14_PutChar(SERVO_START);	// Start byte one
	TX_REPEATER_23_PutChar(SERVO_START);	// Start byte one
	TX_REPEATER_14_PutChar(SERVO_START);	// Start byte two
	TX_REPEATER_23_PutChar(SERVO_START);	// Start byte two
	TX_REPEATER_14_PutChar(id);			// The servo ID
	TX_REPEATER_23_PutChar(id);				// The servo ID
	TX_REPEATER_14_PutChar(length);		// Remaining packet length
	TX_REPEATER_23_PutChar(length);			// Remaining packet length
	TX_REPEATER_14_PutChar(instruction);	// Servo instruction
	TX_REPEATER_23_PutChar(instruction);	// Servo instruction
	TX_REPEATER_14_PutChar(address);		// Target memory address on the servo EEPROM
	TX_REPEATER_23_PutChar(address);		// Target memory address on the servo EEPROM
	TX_REPEATER_14_PutChar(value);			// The write value or number of bytes to read
	TX_REPEATER_23_PutChar(value);			// The write value or number of bytes to read
	TX_REPEATER_14_PutChar(checksum);		// This is the end of this transmission
	TX_REPEATER_23_PutChar(checksum);		// This is the end of this transmission
	
	// Wait for the transmission to finish.
	while(!(TX_REPEATER_14_bReadTxStatus() & TX_REPEATER_14_TX_COMPLETE));
	while(!(TX_REPEATER_23_bReadTxStatus() & TX_REPEATER_23_TX_COMPLETE));
	
	// Make completely sure we're done.
	xmitWait();
}

// This function receives a destination, command length, instruction type, address, and two values.
void longServoInstruction(char id, char length, char instruction, char address, char value1, char value2)
{
	char checksum;	// The checksum byte value.
	int total;		// The total for use in calculating the checksum.
	
	// Get the total of all bytes.
	total = id + length + instruction + address + value1 + value2;
	
	// Calculate the checksum value for our servo communication.
	checksum = 255-(total%256);
	
	// Talk to the servo.
	TX_REPEATER_14_PutChar(SERVO_START);	// Start byte one
	TX_REPEATER_23_PutChar(SERVO_START);	// Start byte one
	TX_REPEATER_14_PutChar(SERVO_START);	// Start byte two
	TX_REPEATER_23_PutChar(SERVO_START);	// Start byte two
	TX_REPEATER_14_PutChar(id);			// The servo ID
	TX_REPEATER_23_PutChar(id);				// The servo ID
	TX_REPEATER_14_PutChar(length);		// Remaining packet length
	TX_REPEATER_23_PutChar(length);			// Remaining packet length
	TX_REPEATER_14_PutChar(instruction);	// Servo instruction
	TX_REPEATER_23_PutChar(instruction);	// Servo instruction
	TX_REPEATER_14_PutChar(address);		// Target memory address on the servo EEPROM
	TX_REPEATER_23_PutChar(address);		// Target memory address on the servo EEPROM
	TX_REPEATER_14_PutChar(value1);		// The first write value
	TX_REPEATER_23_PutChar(value1);			// The first write value
	TX_REPEATER_14_PutChar(value2);		// The second write value
	TX_REPEATER_23_PutChar(value2);			// The second write value
	TX_REPEATER_14_PutChar(checksum);		// This is the end of this transmission
	TX_REPEATER_23_PutChar(checksum);		// This is the end of this transmission
	
	// Wait for the transmission to finish.
	while(!(TX_REPEATER_14_bReadTxStatus() & TX_REPEATER_14_TX_COMPLETE));
	while(!(TX_REPEATER_23_bReadTxStatus() & TX_REPEATER_23_TX_COMPLETE));
	
	// Make completely sure we're done.
	xmitWait();
}

// This function allows the program to pass an RX or TX mode flag for switching between modes on the
// half duplex UART serial communication line.
void configToggle(int mode)
{
	// Disconnect from the global bus and leave the pin high.
	PRT0DR |= 0b11111111;
	PRT0GS &= 0b00000000;

	// Unload the configuration of the current state.
	// If there is no state, blindly wipe all configurations.
	if(STATE)
	{
		unloadConfig(STATE);
	}
	else
	{
		unloadAllConfigs();
	}
	
	if(mode == PC_MODE)
	{
		LoadConfig_pc_listener();

		COMP_SERIAL_CmdReset();								// Initialize the buffer.
		COMP_SERIAL_IntCntl(COMP_SERIAL_ENABLE_RX_INT); 	// Enable RX interrupts  
		COMP_SERIAL_Start(UART_PARITY_NONE);				// Starts the UART.
		
		TX_REPEATER_14_Start(TX_REPEATER_14_PARITY_NONE);	// Start the 014 TX repeater.
		TX_REPEATER_23_Start(TX_REPEATER_23_PARITY_NONE);	// Start the 23 TX repeater.
		
		TIMEOUT = 0;			// Clear the timeout flag.
		TX_TIMEOUT_EnableInt();	// Make sure interrupts are enabled.
		TX_TIMEOUT_Start();		// Start the timer.
		
		// Do nothing while we allow everyone to load the right configuration.
		while(!TIMEOUT){ }
		
		// Stop the timer and reset the timeout flag.
		TX_TIMEOUT_Stop();
		TIMEOUT = 0;
		
		// Store the state.
		STATE = PC_MODE;
	}
	else if(mode == RX_MODE)
	{
		LoadConfig_receiver_config();
		
		// Start the receivers.
		// The seemingly unnecessary brackets around each line are unfortunately needed.
		{
		// Start listening for a response through child port 1.
		RECEIVE_1_Start(RECEIVE_1_PARITY_NONE);
		}
		
		{
		// Start listening for a response through child port 2.
		RECEIVE_2_Start(RECEIVE_2_PARITY_NONE);
		}
		
		{
		// Start listening for a response through child port 3.
		RECEIVE_3_Start(RECEIVE_3_PARITY_NONE);
		}
		
		{
		// Start listening for a response through child port 4.
		RECEIVE_4_Start(RECEIVE_4_PARITY_NONE);
		}
		
		// Start response timeout timer and enable its interrupt routine.
		TIMEOUT = 0;
		RX_TIMEOUT_EnableInt();
		RX_TIMEOUT_Start();
		
		// Store the state.
		STATE = RX_MODE;
	}
	
	// Reconnect to the global bus.
	PRT0GS |= 0b11111111;
}

// This function blindly unloads all user configurations. This will be called once,
// when the system initially has no known state.
void unloadAllConfigs(void)
{
	UnloadConfig_pc_listener();
	UnloadConfig_receiver_config();
}

// This function unloads the configuration corresponding to the config number passed to it.
// We do this instead of unloadAllConfigs to cut down on set up time.
void unloadConfig(int config_num)
{
	if(config_num == PC_MODE)
	{
		UnloadConfig_pc_listener();
	}
	else if(config_num == RX_MODE)
	{
		UnloadConfig_receiver_config();
	}
}

void initializeChildren(void)
{
	int num_timeouts = 0;	// The number of consecutive timeouts.
	int ping_tries = 5;		// The number of times to try a ping on an unregistered module.
	int i = 0;				// An iterator for looping.
	
	// Set num modules to zero.
	NUM_MODULES = 0;
	
	// Set the child value to zero.
	CHILD = 0;	
	
	while(CHILD == 0)
	{
		// Send out a probing message.
		sayHello();
		
		// Listen for a response.
		childListen();
	}
	
	// Send out a probing message.
	sayHello();
	
	// This loop continuously probes and listens at intervals
	// set by the RX_TIMEOUT_DURATION variable.
	while(num_timeouts < MAX_TIMEOUTS)
	{	
		if(validTransmission())
		{
			if(COMMAND_TYPE == HELLO_BYTE)	// Someone else is out there!
			{
				// If this is for me, assign them an ID.
				if(COMMAND_DESTINATION == PARENT_ID)
				{
					NUM_MODULES++;			// Increment the number of modules connected.
					num_timeouts = 0;		// Reset number of timeouts since we found someone.
		
					if(!assignID(NUM_MODULES))
					{
						// If the module did not respond that the ID was assigned,
						// make an effort to ping it in case that transmission was lost
						// before ultimately deciding that the module didn't configure.
						for(i = 0; i < ping_tries; i++)
						{	
							if(pingModule(NUM_MODULES))
							{
								i = ping_tries+1;
							}
						}
						
						// If we landed right at ping_tries, we failed.
						if(i == ping_tries)
						{
							NUM_MODULES--;
						}
					}
				}
			}
		}
		else if(TIMEOUT >= RX_TIMEOUT_DURATION)
		{	
			// Only increment the number of timeouts if we have found a module.
			if(NUM_MODULES)
			{
				num_timeouts++;
			}
			else
			{
				// Wait additional time between transmissions if no modules have been found.
				// This is done to give the first child a chance to configure if it hasn't.
				while(TIMEOUT < INIT_WAIT_TIME) { }
			}
			
			// If we are not maxed out on modules, look for more.
			if(NUM_MODULES < MAX_MODULES)
			{
				sayHello();
			}
		}
	}
	
	// If we didn't find any new modules, check to see if some already exist.
	if(!NUM_MODULES)
	{
		// Try to ping the next module up from our current number ping_tries times.
		for(i = 0; i < ping_tries; i++)
		{	
			if(pingModule(NUM_MODULES+1))
			{
				NUM_MODULES++;
				i = 0;
			}
		}
	}
	
	// Switch back to PC mode.
	configToggle(PC_MODE);
}

// This function listens for children and registers the port that they talk to.
void childListen(void)
{	
	// Wait to either hear a child or time out.
	while(TIMEOUT < RX_TIMEOUT_DURATION)
	{		
		// Check all of the ports for a start byte. Only one port will produce one.
		// Only non-blocking commands are used to avoid getting stuck listening downstream.
		if(RECEIVE_1_cReadChar() == START_TRANSMIT)
		{
			while(TIMEOUT < RX_TIMEOUT_DURATION)
			{
				if(RECEIVE_1_cReadChar() == END_TRANSMIT)
				{
					CHILD = PORT_1;
				}
			}
		}
		else if(RECEIVE_2_cReadChar() == START_TRANSMIT)
		{
			while(TIMEOUT < RX_TIMEOUT_DURATION)
			{
				if(RECEIVE_2_cReadChar() == END_TRANSMIT)
				{
					CHILD = PORT_2;
				}
			}
		}
		else if(RECEIVE_3_cReadChar() == START_TRANSMIT)
		{
			while(TIMEOUT < RX_TIMEOUT_DURATION)
			{
				if(RECEIVE_3_cReadChar() == END_TRANSMIT)
				{
					CHILD = PORT_3;
				}
			}
		}
		else if(RECEIVE_4_cReadChar() == START_TRANSMIT)
		{
			while(TIMEOUT < RX_TIMEOUT_DURATION)
			{
				if(RECEIVE_4_cReadChar() == END_TRANSMIT)
				{
					CHILD = PORT_4;
				}
			}
		}
	}
}

// This function converts the PSoC cReadChar calls of all ports into a single return.
char iReadChar(void)
{
	if(CHILD == PORT_1)
	{
		return RECEIVE_1_cReadChar();
	}
	else if(CHILD == PORT_2)
	{
		return RECEIVE_2_cReadChar();
	}
	else if(CHILD == PORT_3)
	{
		return RECEIVE_3_cReadChar();
	}
	else if(CHILD == PORT_4)
	{
		return RECEIVE_4_cReadChar();
	}
	else
	{
		return 0;
	}
}

// This function converts the PSoC cGetChar calls of all ports into a single return.
char readChar(void)
{	
	if(CHILD == PORT_1)
	{
		return RECEIVE_1_cGetChar();
	}
	else if(CHILD == PORT_2)
	{
		return RECEIVE_2_cGetChar();
	}
	else if(CHILD == PORT_3)
	{
		return RECEIVE_3_cGetChar();
	}
	else if(CHILD == PORT_4)
	{
		return RECEIVE_4_cGetChar();
	}
	else
	{
		return 0;
	}
}

void xmitWait(void)
{
	int i;
	
	for(i = 0; i < 25; i++)
	{
		// Sit here and spin for about 50 microseconds.
	}
}

void TX_TIMEOUT_ISR(void)
{	
	// Increment the number of timeouts.
	TIMEOUT++;
	
	M8C_ClearIntFlag(INT_CLR0,TX_TIMEOUT_INT_MASK);
}

void RX_TIMEOUT_ISR(void)
{	
	// Increment the number of timeouts.
	TIMEOUT++;
	
	M8C_ClearIntFlag(INT_CLR0,RX_TIMEOUT_INT_MASK);
}