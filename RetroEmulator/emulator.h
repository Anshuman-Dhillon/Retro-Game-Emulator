// emulator.h - Game Boy Emulator Header
// This file defines the main emulator class that handles CPU, memory, and graphics
#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <deque>

class Emulator {
public:
    Emulator();
    ~Emulator();

    bool running = false;        // Whether the emulator is currently executing

    // ROM and program loading
    bool loadROM(const std::string& filename);  // Load a .gb ROM file from disk
    void loadTestProgram();                     // Load a simple test program for debugging
    void loadTestPattern();                     // Load a simple test program for debugging

    void reset();                              // Reset CPU and memory to initial state

	void debugTileData();                   // Debug function to visualize tile data
    void debugRenderingState();
    void debugSpecificTile(int tile_index);
    void validateVRAMData();
    void debug8800MethodTiles();

    // Execution control
    void step();                               // Execute exactly one CPU instruction
    void runFrame();                          // Run until one video frame is complete (~70224 cycles)

    // Debug/ImGui interface structures
    // These expose internal state for the debugging GUI
    struct CPUState {
        uint16_t PC, SP;    // Program Counter (current instruction) and Stack Pointer
        uint8_t A, B, C, D, E, H, L;  // 8-bit registers (A is accumulator, others are general purpose)
        uint8_t F;          // Flags register (Zero, Subtract, Half-carry, Carry flags)
        bool ime;           // Interrupt Master Enable flag
        uint64_t cycles;    // Total cycles executed since start
    } cpu;

    // Picture Processing Unit (graphics chip) state
    struct PPUState {
        uint8_t lcdc, stat; // LCD Control and Status registers
        uint8_t scy, scx;   // Background scroll Y and X
        uint8_t ly, lyc;    // Current scanline and scanline compare
        uint8_t wy, wx;     // Window Y and X position
        uint8_t bgp, obp0, obp1; // Background and sprite palettes
        bool vblank_flag;   // True when in vertical blank period
    } ppu;

    // Joypad/Controller state
    struct JoypadState {
        bool up, down, left, right;      // D-pad
        bool a, b, select, start;        // Buttons
    } joypad;

    uint8_t memory[0x10000];     // 64KB address space (Game Boy memory map)

    // PPU timing variables
    int cycles_in_scanline = 0;  // Cycles elapsed in current scanline
    int cycles_since_last_update = 0; // For timing calculations

    // Memory access functions for debugging
    uint8_t readMemory(uint16_t addr) const;   // Read one byte from any memory address
    void writeMemory(uint16_t addr, uint8_t value); // Write one byte to any memory address

    // Screen buffer - represents the Game Boy's 160x144 pixel LCD screen
    // Each pixel is 2 bits (4 shades of gray: 0=white, 1=light gray, 2=dark gray, 3=black)
    uint8_t screen[160 * 144];
    uint8_t vram[0x2000];       // Video RAM (0x8000-0x9FFF)
    std::vector<uint8_t> rom;    // The loaded ROM file data

    uint16_t div_counter;
    uint16_t timer_counter;

    // Execution state control
    bool isRunning() const { return running; }
    void pause() { running = false; }
    void resume() { running = true; }

    // Helper functions for flag manipulation
    void setZeroFlag(bool condition) {
        cpu.F = condition ? (cpu.F | 0x80) : (cpu.F & 0x7F);
    }
    void setSubtractFlag(bool condition) {
        cpu.F = condition ? (cpu.F | 0x40) : (cpu.F & 0xBF);
    }
    void setHalfCarryFlag(bool condition) {
        cpu.F = condition ? (cpu.F | 0x20) : (cpu.F & 0xDF);
    }
    void setCarryFlag(bool condition) {
        cpu.F = condition ? (cpu.F | 0x10) : (cpu.F & 0xEF);
    }

    // Flag checking functions
    bool getZeroFlag() const { return (cpu.F & 0x80) != 0; }
    bool getSubtractFlag() const { return (cpu.F & 0x40) != 0; }
    bool getHalfCarryFlag() const { return (cpu.F & 0x20) != 0; }
    bool getCarryFlag() const { return (cpu.F & 0x10) != 0; }

private:
    // Core emulator data

    // Separate memory regions for proper memory mapping
    uint8_t wram[0x2000];       // Work RAM (0xC000-0xDFFF)
    uint8_t oam[0xA0];          // Object Attribute Memory - sprite data
    uint8_t hram[0x7F];         // High RAM (0xFF80-0xFFFE)

    static const int OAM_CYCLES = 80;
    static const int PIXEL_TRANSFER_CYCLES = 172;  
    static const int HBLANK_CYCLES = 204;
    static const int SCANLINE_CYCLES = 456;
    static const int VBLANK_LINES = 10;
    static const int TOTAL_LINES = 154;

    enum class MBCType { NONE, MBC1 };
    MBCType mbc_type = MBCType::NONE;
    uint8_t mbc1_rom_bank = 1;      // Current ROM bank (1-127)
    uint8_t mbc1_ram_bank = 0;      // Current RAM bank (0-3)
    bool mbc1_ram_enable = false;   // RAM enabled
    bool mbc1_mode = false;         // false = ROM banking, true = RAM banking
    std::vector<uint8_t> mbc1_ram;  // External RAM (if present)

    // Internal emulation methods (these do the actual work)
    void executeInstruction();   // Decode and execute one CPU instruction
    void updateTimers();        // Update the Game Boy's internal timers
    void updateGraphics();      // Update the PPU and render graphics
    void handleInterrupts();    // Process interrupt requests

    // PPU helper functions
    void renderScanline(int line);  // Render one scanline of graphics

    // Memory mapping helper functions
    uint8_t readIORegister(uint16_t addr) const; // Read from I/O registers (0xFF00-0xFF7F)
    void writeIORegister(uint16_t addr, uint8_t value); // Write to I/O registers

    // Stack operations for interrupts and function calls
    void pushStackWord(uint16_t value);   // Push 16-bit value onto stack
    uint16_t popStackWord();              // Pop 16-bit value from stack

    // Joypad register handling
    uint8_t readJoypadRegister() const;   // Read joypad state (0xFF00)

    // Arithmetic helper functions
    void addToA(uint8_t value, bool carry = false);    // Add to accumulator with flags
    void subFromA(uint8_t value, bool carry = false);  // Subtract from accumulator with flags
    void compareA(uint8_t value);                       // Compare with accumulator
    void andA(uint8_t value);                          // Bitwise AND with accumulator
    void orA(uint8_t value);                           // Bitwise OR with accumulator
    void xorA(uint8_t value);                          // Bitwise XOR with accumulator
};