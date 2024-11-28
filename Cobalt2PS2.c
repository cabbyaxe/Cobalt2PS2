/**
 * @file Cobalt2PS2.c
 * @brief Enables raspberry pi pico to act as a cobalt flux control box for PS2 games
 * 
 * This module translates button presses from the Cobalt Flux dance pad into PlayStation 2 controller inputs.
 * It uses a raspberry pi pico 
 */

#ifndef Cobalt2PS2_H
#define Cobalt2PS2_H

#include "pico/multicore.h"
#include "hardware/structs/sio.h"
#include <stdio.h>          // Standard I/O operations
#include "pico/stdlib.h"    // Raspberry Pi Pico standard library
#include "hardware/pio.h"   // Programmable I/O (PIO) functionality
#include "hardware/irq.h"   // Interrupt handling
#include "psxSPI.pio.h"     // PIO program for PS2 SPI protocol
#include "hardware/sync.h"  // Add this header for interrupt control
#include "pico/mutex.h"


/**
 * @brief GPIO pin definitions for Cobalt dance pad connections
 * 
 * These constants define the GPIO pin numbers where the Cobalt dance pad
 * sensors are connected on the microcontroller.
 *
 * @def PAD_UP Pin for up arrow sensor (0)
 * @def PAD_DOWN Pin for down arrow sensor (1) 
 * @def PAD_LEFT Pin for left arrow sensor (2)
 * @def PAD_RIGHT Pin for right arrow sensor (3)
 * @def PAD_TOPRIGHT Pin for top-right arrow sensor (10)
 * @def PAD_TOPLEFT Pin for top-left arrow sensor (4)
 * @def PAD_BOTLEFT Pin for bottom-left arrow sensor (11)
 * @def PAD_BOTRIGHT Pin for bottom-right arrow sensor (12)
 */
#define PAD_UP    0
#define PAD_DOWN  1
#define PAD_LEFT  2
#define PAD_RIGHT 3
#define PAD_TOPRIGHT 10
#define PAD_TOPLEFT 4
#define PAD_BOTLEFT 11
#define PAD_BOTRIGHT 12


/* Protocol Constants
 * The PS2 controller identification range (0x41-0x5A)
 * Used during the controller identification phase
 */
#define ID_LO 0x41    // Digital controller ID lower bound
#define ID_HI 0x5A    // Digital controller ID upper bound

/* Button State Constants
 * Bit patterns representing different button states
 * These constants are used to simulate button presses on the controller
 */
#define R_PRESSED 0b11011111  
#define L_PRESSED 0b01111111  
#define NO_PRESSED 0xff        
#define SEND_ACK 0b100000000  
#define BTN_SELECT      (1 << 0)
#define BTN_L3         (1 << 1)
#define BTN_R3         (1 << 2)
#define BTN_START      (1 << 3)
#define BTN_UP    0b00010000
#define BTN_RIGHT 0b00100000
#define BTN_DOWN  0b01000000
#define BTN_LEFT 0b10000000
#define BTN_L2         (1 << 8)
#define BTN_R2         (1 << 9)
#define BTN_L1         (1 << 10)
#define BTN_R1         (1 << 11)
#define BTN_TRIANGLE    (1 << 12)
#define BTN_CIRCLE     (1 << 13)
#define BTN_CROSS      (1 << 14)
#define BTN_SQUARE     (1 << 15)


/**
 * @brief Controller mode definitions
 *
 * Defines constants for different controller operating modes:
 * - MODE_DIGITAL (0x41): Digital input mode
 * - MODE_ANALOG (0x73): Analog input mode
 */
#define MODE_DIGITAL 0x41
#define MODE_ANALOG  0x73

//stdio mutex for uart out / in on core1
static mutex_t stdio_mutex;

// Thread-safe button state
static volatile uint16_t current_button_state = 0xff;
static volatile bool new_state_available = false;

/* PIO Configuration */
static const PIO pio = pio0;  // Using PIO0 instance

/* State Machine IDs */
static uint smSelMonitor;
static uint smCmdReader;
static uint smDatWriter;

/* Program Offsets */
static uint offsetSelMonitor;
static uint offsetCmdReader;
static uint offsetDatWriter;

/**
 * @brief Current operation mode of the controller.
 * 
 * Stores the current mode of operation for the PS2 controller.
 * Can be either digital (default) or analog mode.
 */
static uint8_t current_mode = MODE_DIGITAL;

/**
 * @brief Previous state structure for directional input tracking
 * 
 * Static structure maintaining the previous state of 8-way directional inputs.
 * Used to track the last known state of directional controls.
 * 
 * @note All fields are boolean flags representing pressed (true) or released (false)
 */
static struct {
	bool up;
	bool down;
	bool left;
	bool right;
	bool topright;
	bool topleft;
	bool botleft;
	bool botright;
} prev_state = {0};

/**
 * @brief Validates if state machine ID is within valid range
 * @param sm_id State machine ID to validate
 * @return true if valid, false otherwise
 */
static inline bool validate_sm_id(uint sm_id) {
    return sm_id < NUM_PIO_STATE_MACHINES;
}

/**
 * @brief Interrupt handler called when SEL goes high
 * Resets cmdReader and datWriter state machines
 */
void __not_in_flash_func(pio0_irq0)(void) {
    if (!validate_sm_id(smCmdReader) || !validate_sm_id(smDatWriter)) {
        return;  // Invalid state machine IDs
    }

    uint32_t sm_mask = (1u << smCmdReader) | (1u << smDatWriter);
    
    // Disable state machines
    pio_set_sm_mask_enabled(pio, sm_mask, false);
    
    // Reset state machines
    pio_restart_sm_mask(pio, sm_mask);
    
    // Reset program counters
    pio_sm_exec(pio, smCmdReader, pio_encode_jmp(offsetCmdReader));
    pio_sm_exec(pio, smDatWriter, pio_encode_jmp(offsetDatWriter));
    
    // Re-enable state machines synchronously
    pio_enable_sm_mask_in_sync(pio, sm_mask);
    
    // Clear interrupt
    pio_interrupt_clear(pio0, 0);
}


/**
 * @brief Initializes the dance pad GPIO pins
 * 
 * This function configures all dance pad buttons as input pins with pull-up resistors enabled.
 * It handles initialization for 8 buttons:
 * - Up, Down, Left, Right (cardinal directions)
 * - Top-left, Top-right, Bottom-left, Bottom-right (diagonal directions)
 * 
 * The initialization process consists of three steps for each pin:
 * 1. Basic GPIO initialization
 * 2. Setting direction as input
 * 3. Enabling the internal pull-up resistor
 * 
 */
void init_dance_pad(void) {
	gpio_init(PAD_UP);
	gpio_init(PAD_DOWN);
	gpio_init(PAD_LEFT); 
	gpio_init(PAD_RIGHT);
	gpio_init(PAD_TOPLEFT);
	gpio_init(PAD_TOPRIGHT);
	gpio_init(PAD_BOTLEFT);
	gpio_init(PAD_BOTRIGHT);
	
	gpio_set_dir(PAD_UP, GPIO_IN);
	gpio_set_dir(PAD_DOWN, GPIO_IN);
	gpio_set_dir(PAD_LEFT, GPIO_IN);
	gpio_set_dir(PAD_RIGHT, GPIO_IN);
	gpio_set_dir(PAD_TOPLEFT, GPIO_IN);
	gpio_set_dir(PAD_TOPRIGHT, GPIO_IN);
	gpio_set_dir(PAD_BOTLEFT, GPIO_IN);
	gpio_set_dir(PAD_BOTRIGHT, GPIO_IN);
	
	gpio_pull_up(PAD_UP);
	gpio_pull_up(PAD_DOWN);
	gpio_pull_up(PAD_LEFT);
	gpio_pull_up(PAD_RIGHT);
	gpio_pull_up(PAD_TOPLEFT);
	gpio_pull_up(PAD_TOPRIGHT);
	gpio_pull_up(PAD_BOTLEFT);
	gpio_pull_up(PAD_BOTRIGHT);
}

/**
 * @brief Switches the current controller mode based on button inputs atomically.
 * 
 * This function performs an atomic mode switch operation by temporarily
 * disabling interrupts during the mode change to ensure thread safety.
 * 
 * @param mode The new controller mode to switch to
 * 
 * @note This function briefly disables interrupts to ensure atomic operation
 */
void switch_controller_mode(uint8_t mode) {
    uint32_t save = save_and_disable_interrupts();
    
    // Check if X or O is pressed
    if (!(current_button_state & (BTN_CROSS | BTN_CIRCLE))) {
        current_mode = MODE_ANALOG;  // Switch to analog if X or O pressed
    } else {
        current_mode = MODE_DIGITAL; // Otherwise stay in digital
    }
    
    restore_interrupts(save);
}

/**
 * @brief Handles the PS2 controller command sequence and responds with appropriate data
 * 
 * This function implements the PS2 controller communication protocol by:
 * 1. Sending controller ID bytes
 * 2. Verifying command sequence (0x42, 0x00)
 * 3. Sending button states and analog values
 * 4. Handling pressure-sensitive buttons in analog mode
 *
 * The function follows the standard PS2 controller polling sequence:
 * - Sends controller ID (low byte then high byte)
 * - Sends current button states
 * - In analog mode, sends stick positions and pressure values
 * - Terminates sequence with final status bytes
 *
 * @param next_byte The command byte received from the PS2 console
 * 
 * @note The function uses blocking writes/reads for communication
 * @note In analog mode, stick values default to centered (0x80)
 * @note Pressure values default to 0xFF (no pressure) except for X and O buttons
 */
void handle_ps2_controller_command(uint8_t next_byte) {
	// Send the lower controller ID byte
	write_byte_blocking(pio, smDatWriter, ID_LO);

	// Read next command byte - expect 0x42
	next_byte = read_byte_blocking(pio, smCmdReader);
	if(next_byte != 0x42) {
		return;
	} else {
		// Send the upper controller ID byte
		write_byte_blocking(pio, smDatWriter, ID_HI);
	}

	// Read next command byte - expect 0x00
	next_byte = read_byte_blocking(pio, smCmdReader);
	if(next_byte != 0x00) {
		return;
	} else {
		switch_controller_mode(current_mode);
		write_byte_blocking(pio, smDatWriter, (current_button_state >> 8) & 0xFF);
		write_byte_blocking(pio, smDatWriter, current_button_state & 0xFF);

		if (current_mode == MODE_ANALOG) {
			// Analog stick values (centered)
			write_byte_blocking(pio, smDatWriter, 0x80);    // RX
			write_byte_blocking(pio, smDatWriter, 0x80);    // RY
			write_byte_blocking(pio, smDatWriter, 0x80);    // LX
			write_byte_blocking(pio, smDatWriter, 0x80);    // LY
			
			// Button pressure values
			for(int i = 0; i < 12; i++) {
				uint8_t pressure = 0xFF;  // Default no pressure
				
				// Set full pressure (0xFF) for X or O when pressed
				if (i == 6 && !(current_button_state & BTN_CROSS)) {  // X button
					pressure = 0xFF;
				}
				if (i == 5 && !(current_button_state & BTN_CIRCLE)) { // O button
					pressure = 0xFF;
				}
				
				write_byte_blocking(pio, smDatWriter, pressure);
			}
		}
	}

	// Read next command byte - expect 0x00
	next_byte = read_byte_blocking(pio, smCmdReader);
	if(next_byte != 0x00) return;
	write_byte_blocking(pio, smDatWriter, NO_PRESSED);

	// Read final command byte - expect 0x00
	next_byte = read_byte_blocking(pio, smCmdReader);
	if(next_byte != 0x00) return;
}



/**
 * @brief Continuously reads and processes the dance pad state
 * 
 * This function initializes the dance pad and enters an infinite loop to:
 * 1. Read the current state of all dance pad inputs (inverted due to pull-up configuration)
 * 2. Update the global button state based on pad inputs
 * 3. Print state changes to console (protected by mutex)
 * 4. Apply debounce delay
 * 
 * The function maps dance pad inputs to PS2 controller buttons:
 * - Up pad -> Up button
 * - Down pad -> Down button  
 * - Left pad -> Left button
 * - Right pad -> Right button
 * - Top Left pad -> Triangle button
 * - Top Right pad -> Cross button
 * - Bottom Left pad -> Select button
 * - Bottom Right pad -> Start button
 */
void read_cobalt_state() {
	init_dance_pad();
	
	while (1) {
		// Read current states (inverted because pull-up)
		bool up = !gpio_get(PAD_UP);
		bool down = !gpio_get(PAD_DOWN);
		bool left = !gpio_get(PAD_LEFT);
		bool right = !gpio_get(PAD_RIGHT);
		bool topleft = !gpio_get(PAD_TOPLEFT);
		bool topright = !gpio_get(PAD_TOPRIGHT);
		bool botleft = !gpio_get(PAD_BOTLEFT);
		bool botright = !gpio_get(PAD_BOTRIGHT);
		
		// Update button state based on pad inputs
		uint16_t new_state = 0xFFFF; // Start with all buttons released

		if (up) new_state &= ~BTN_UP;
		if (down) new_state &= ~BTN_DOWN;
		if (left) new_state &= ~BTN_LEFT; 
		if (right) new_state &= ~BTN_RIGHT;
		if (topleft) new_state &= ~BTN_TRIANGLE;
		if (topright) new_state &= ~BTN_CROSS;
		if (botleft) new_state &= ~BTN_SELECT;
		if (botright) new_state &= ~BTN_START;

		// Update global button state
		current_button_state = new_state;
		
		// Check for changes and print
		mutex_enter_blocking(&stdio_mutex);
		if (up != prev_state.up) {
			printf("Up: %s\n", up ? "Pressed" : "Released");
			prev_state.up = up;
		}
		if (down != prev_state.down) {
			printf("Down: %s\n", down ? "Pressed" : "Released");
			prev_state.down = down;
		}
		if (left != prev_state.left) {
			printf("Left: %s\n", left ? "Pressed" : "Released");
			prev_state.left = left;
		}
		if (right != prev_state.right) {
			printf("Right: %s\n", right ? "Pressed" : "Released");
			prev_state.right = right;
		}
		if (topleft != prev_state.topleft) {
			printf("Top Left: %s\n", topleft ? "Pressed" : "Released");
			prev_state.topleft = topleft;
		}
		if (topright != prev_state.topright) {
			printf("Top Right: %s\n", topright ? "Pressed" : "Released");
			prev_state.topright = topright;
		}
		if (botleft != prev_state.botleft) {
			printf("Bottom Left: %s\n", botleft ? "Pressed" : "Released");
			prev_state.botleft = botleft;
		}
		if (botright != prev_state.botright) {
			printf("Bottom Right: %s\n", botright ? "Pressed" : "Released");
			prev_state.botright = botright;
		}
		mutex_exit(&stdio_mutex);

		// debounce delay
		sleep_ms(20);
	}
}

#endif // Cobalt2PS2_H


int main() {

	stdio_init_all();

	/* Initialize PS2 Controller Interface */
	/* Setup PIO interrupts */
	irq_set_exclusive_handler(PIO0_IRQ_0, pio0_irq0);
	irq_set_enabled(PIO0_IRQ_0, true);
	offsetCmdReader = pio_add_program(pio, &cmd_reader_program);
	offsetDatWriter = pio_add_program(pio, &dat_writer_program);
	smSelMonitor = pio_claim_unused_sm(pio, true);
	smCmdReader = pio_claim_unused_sm(pio, true);
	smDatWriter = pio_claim_unused_sm(pio, true);
	dat_writer_program_init(pio, smDatWriter, offsetDatWriter);
	cmd_reader_program_init(pio, smCmdReader, offsetCmdReader);
	uint32_t smMask = (1 << smSelMonitor) | (1 << smCmdReader) | (1 << smDatWriter);
	pio_enable_sm_mask_in_sync(pio, smMask);

	// Launch cobalt flux dance pad reading on core1
    multicore_launch_core1(read_cobalt_state);

	//init stdio_mutex
	mutex_init(&stdio_mutex);

	// Main command processing loop that continuously reads PS2 controller commands
	while(true) {
		uint8_t item = read_byte_blocking(pio, smCmdReader);
		if(item == 0x01) {
			handle_ps2_controller_command(item);
		}
	}
}

