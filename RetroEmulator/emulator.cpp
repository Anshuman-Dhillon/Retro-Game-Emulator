// emulator.cpp - Game Boy Emulator Core Implementation
// This file implements the core emulation logic
#include "emulator.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <map>

// Constructor - initialize the emulator
Emulator::Emulator() {
    // Initialize all memory regions to zero
    memset(memory, 0, sizeof(memory));
    memset(vram, 0, sizeof(vram));
    memset(wram, 0, sizeof(wram));
    memset(oam, 0, sizeof(oam));
    memset(hram, 0, sizeof(hram));
    memset(screen, 0, sizeof(screen));

    // Initialize joypad state (all buttons released)
    memset(&joypad, 0, sizeof(joypad));

    reset();
}

// Destructor - cleanup (nothing special needed for now)
Emulator::~Emulator() {}

// Load a Game Boy ROM file (.gb or .gbc)
bool Emulator::loadROM(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open ROM: " << filename << std::endl;
        return false;
    }

    rom.assign(std::istreambuf_iterator<char>(file),
        std::istreambuf_iterator<char>());

    // --- MBC1 detection and RAM allocation ---
    mbc_type = MBCType::NONE;
    mbc1_rom_bank = 1;
    mbc1_ram_bank = 0;
    mbc1_ram_enable = false;
    mbc1_mode = false;
    mbc1_ram.clear();

    if (rom.size() > 0x8000) {
        uint8_t cart_type = rom[0x147];
        if (cart_type == 0x01 || cart_type == 0x02 || cart_type == 0x03) {
            mbc_type = MBCType::MBC1;
            // RAM size from header
            uint8_t ram_size = rom[0x149];
            size_t ram_bytes = 0;
            if (ram_size == 0x01) ram_bytes = 2 * 1024;
            else if (ram_size == 0x02) ram_bytes = 8 * 1024;
            else if (ram_size == 0x03) ram_bytes = 32 * 1024;
            if (ram_bytes) mbc1_ram.resize(ram_bytes, 0);
        }
    }

    reset();
    running = true;

    std::cout << "Loaded ROM: " << filename << " (" << rom.size() << " bytes)";
    if (mbc_type == MBCType::MBC1) std::cout << " [MBC1 detected]";
    std::cout << std::endl;
    return true;
}

// Reset the Game Boy to its initial power-on state
void Emulator::reset() {
    // CORRECT Game Boy boot-up values (these were wrong in your original)
    cpu.PC = 0x0100;
    cpu.SP = 0xFFFE;
    cpu.A = 0x01;   // Original Game Boy
    cpu.B = 0x00; cpu.C = 0x13;
    cpu.D = 0x00; cpu.E = 0xD8;
    cpu.H = 0x01; cpu.L = 0x4D;
    cpu.F = 0xB0;   // Zero and Half-carry flags set
    cpu.ime = true; // CRITICAL: Interrupts should be enabled after boot
    cpu.cycles = 0;

    // CORRECT PPU initial state
    ppu.lcdc = 0x91;    // LCD on, BG on, sprites on
    ppu.stat = 0x02;    // Mode 2 (OAM search) - CRITICAL for timing tests
    ppu.scy = ppu.scx = 0x00;
    ppu.ly = 0x00;      // Start at line 0
    ppu.lyc = 0x00;
    ppu.wy = ppu.wx = 0x00;
    ppu.bgp = 0xFC;     // Correct default palette
    ppu.obp0 = ppu.obp1 = 0xFF;
    ppu.vblank_flag = false;

    // CRITICAL: Initialize I/O registers with CORRECT default values
    memset(memory, 0, sizeof(memory));

    // These exact values are critical for some tests
    memory[0xFF00] = 0xCF; // Joypad
    memory[0xFF04] = 0x00; // DIV
    memory[0xFF05] = 0x00; // TIMA
    memory[0xFF06] = 0x00; // TMA
    memory[0xFF07] = 0x00; // TAC
    memory[0xFF0F] = 0x00; // IF (no interrupts pending)
    memory[0xFF40] = 0x91; // LCDC
    memory[0xFF41] = 0x02; // STAT (mode 2)
    memory[0xFF42] = 0x00; // SCY
    memory[0xFF43] = 0x00; // SCX
    memory[0xFF44] = 0x00; // LY
    memory[0xFF45] = 0x00; // LYC
    memory[0xFF47] = 0xFC; // BGP
    memory[0xFF48] = 0xFF; // OBP0
    memory[0xFF49] = 0xFF; // OBP1
    memory[0xFF4A] = 0x00; // WY
    memory[0xFF4B] = 0x00; // WX
    memory[0xFFFF] = 0x00; // IE (all interrupts disabled initially)

    // Initialize timing
    cycles_in_scanline = 0;
    cycles_since_last_update = 0;

    // Clear work areas
    memset(vram, 0, sizeof(vram));
    memset(wram, 0, sizeof(wram));
    memset(oam, 0, sizeof(oam));
    memset(hram, 0, sizeof(hram));
    memset(screen, 0, sizeof(screen));
}

// Execute one CPU instruction, then update timers, graphics, and interrupts
void Emulator::step() {
    if (!running) return;

    uint64_t cycles_before = cpu.cycles;

    // Execute one instruction
    executeInstruction();

    // Calculate how many cycles this instruction took
    cycles_since_last_update = (int)(cpu.cycles - cycles_before);

    // CRITICAL: Update subsystems in the correct order
    updateTimers();       // Update timers first
    updateGraphics();     // Then graphics (PPU)
    handleInterrupts();   // Finally handle interrupts
}

// Run the emulator for one complete video frame
// Game Boy runs at ~59.7 FPS, so one frame = ~70224 CPU cycles
void Emulator::runFrame() {
    if (!running) return;

    ppu.vblank_flag = false;  // Clear the frame-complete flag

    // Keep executing instructions until we complete one frame
    while (!ppu.vblank_flag && running) {
        step();
    }
}

// Read one byte from the Game Boy's memory map
uint8_t Emulator::readMemory(uint16_t addr) const {
    // Add memory access timing
    // Different memory regions have different access times

    if (addr < 0x4000) {
        // ROM Bank 0 - fast access
        return rom.size() > addr ? rom[addr] : 0xFF;
    }
    else if (addr < 0x8000) {
        // Switchable ROM bank
        if (mbc_type == MBCType::MBC1) {
            size_t bank = mbc1_rom_bank & 0x7F;
            if (bank == 0) bank = 1;
            size_t offset = bank * 0x4000 + (addr - 0x4000);
            return rom.size() > offset ? rom[offset] : 0xFF;
        }
        else {
            return rom.size() > addr ? rom[addr] : 0xFF;
        }
    }
    else if (addr >= 0x8000 && addr < 0xA000) {
        // VRAM - can be blocked during pixel transfer
        if ((ppu.stat & 0x03) == 0x03) {
            // Mode 3 - VRAM inaccessible
            return 0xFF;
        }
        return vram[addr - 0x8000];
    }
    else if (addr >= 0xA000 && addr < 0xC000) {
        // External RAM
        if (mbc_type == MBCType::MBC1 && mbc1_ram_enable && !mbc1_ram.empty()) {
            size_t ram_addr = addr - 0xA000;
            size_t bank = mbc1_mode ? mbc1_ram_bank : 0;
            size_t offset = bank * 0x2000 + ram_addr;
            if (offset < mbc1_ram.size()) return mbc1_ram[offset];
        }
        return 0xFF;
    }
    else if (addr >= 0xC000 && addr < 0xE000) {
        // Work RAM
        return wram[addr - 0xC000];
    }
    else if (addr >= 0xE000 && addr < 0xFE00) {
        // Echo RAM
        return wram[(addr - 0xE000) % 0x1E00];
    }
    else if (addr >= 0xFE00 && addr < 0xFEA0) {
        // OAM - can be blocked during OAM search and pixel transfer
        if ((ppu.stat & 0x03) >= 0x02) {
            // Mode 2 or 3 - OAM inaccessible
            return 0xFF;
        }
        return oam[addr - 0xFE00];
    }
    else if (addr >= 0xFEA0 && addr < 0xFF00) {
        // Unusable
        return 0xFF;
    }
    else if (addr >= 0xFF00 && addr < 0xFF80) {
        // I/O registers
        return readIORegister(addr);
    }
    else if (addr >= 0xFF80 && addr < 0xFFFF) {
        // High RAM
        return hram[addr - 0xFF80];
    }
    else if (addr == 0xFFFF) {
        // Interrupt Enable
        return memory[0xFFFF];
    }

    return 0xFF;
}

// Write one byte to the Game Boy's memory map  
void Emulator::writeMemory(uint16_t addr, uint8_t value) {
    // Handle MBC writes first (before 0x8000)
    if (addr < 0x2000 && mbc_type == MBCType::MBC1) {
        mbc1_ram_enable = ((value & 0x0F) == 0x0A);
        return;
    }
    else if (addr >= 0x2000 && addr < 0x4000 && mbc_type == MBCType::MBC1) {
        mbc1_rom_bank = (mbc1_rom_bank & 0x60) | (value & 0x1F);
        if ((mbc1_rom_bank & 0x1F) == 0) mbc1_rom_bank |= 1;
        return;
    }
    else if (addr >= 0x4000 && addr < 0x6000 && mbc_type == MBCType::MBC1) {
        if (mbc1_mode) {
            mbc1_ram_bank = value & 0x03;
        }
        else {
            mbc1_rom_bank = (mbc1_rom_bank & 0x1F) | ((value & 0x03) << 5);
            if ((mbc1_rom_bank & 0x1F) == 0) mbc1_rom_bank |= 1;
        }
        return;
    }
    else if (addr >= 0x6000 && addr < 0x8000 && mbc_type == MBCType::MBC1) {
        mbc1_mode = (value & 0x01) != 0;
        return;
    }
    else if (addr < 0x8000) {
        return; // ROM area
    }
    else if (addr >= 0x8000 && addr < 0xA000) {
        // VRAM - check if accessible
        if ((ppu.stat & 0x03) == 0x03) {
            // Mode 3 - VRAM blocked
            return;
        }
        vram[addr - 0x8000] = value;
    }
    else if (addr >= 0xFE00 && addr < 0xFEA0) {
        // OAM - check if accessible
        if ((ppu.stat & 0x03) >= 0x02) {
            // Mode 2 or 3 - OAM blocked
            return;
        }
        oam[addr - 0xFE00] = value;
    }
    else if (addr >= 0xFF00 && addr < 0xFF80) {
        // I/O registers
        writeIORegister(addr, value);
    }
    else if (addr >= 0xFF80 && addr < 0xFFFF) {
        // High RAM
        hram[addr - 0xFF80] = value;
    }
    else if (addr == 0xFFFF) {
        // Interrupt Enable
        memory[0xFFFF] = value;
    }
    else if (addr >= 0xC000 && addr < 0xE000) {
        // Work RAM
        wram[addr - 0xC000] = value;
    }
    else if (addr >= 0xE000 && addr < 0xFE00) {
        // Echo RAM
        wram[(addr - 0xE000) % 0x1E00] = value;
    }
    else if (addr >= 0xA000 && addr < 0xC000) {
        // External RAM
        if (mbc_type == MBCType::MBC1 && mbc1_ram_enable && !mbc1_ram.empty()) {
            size_t ram_addr = addr - 0xA000;
            size_t bank = mbc1_mode ? mbc1_ram_bank : 0;
            size_t offset = bank * 0x2000 + ram_addr;
            if (offset < mbc1_ram.size()) mbc1_ram[offset] = value;
        }
    }
}

// Read from I/O registers (0xFF00-0xFF7F)
uint8_t Emulator::readIORegister(uint16_t addr) const {
    switch (addr) {
    case 0xFF00: // Joypad register
        return readJoypadRegister();
    case 0xFF04: // DIV register (divider)
        return memory[addr];
    case 0xFF05: // TIMA register (timer counter)
        return memory[addr];
    case 0xFF06: // TMA register (timer modulo)
        return memory[addr];
    case 0xFF07: // TAC register (timer control)
        return memory[addr];
    case 0xFF0F: // IF register (interrupt flags)
        return memory[addr];
    case 0xFF40: // LCDC register
        return ppu.lcdc;
    case 0xFF41: // STAT register
        return ppu.stat;
    case 0xFF42: // SCY register
        return ppu.scy;
    case 0xFF43: // SCX register
        return ppu.scx;
    case 0xFF44: // LY register (current scanline)
        return ppu.ly;
    case 0xFF45: // LYC register (scanline compare)
        return ppu.lyc;
    case 0xFF47: // BGP register (background palette)
        return ppu.bgp;
    case 0xFF48: // OBP0 register (sprite palette 0)
        return ppu.obp0;
    case 0xFF49: // OBP1 register (sprite palette 1)
        return ppu.obp1;
    case 0xFF4A: // WY register (window Y)
        return ppu.wy;
    case 0xFF4B: // WX register (window X)
        return ppu.wx;
    default:
        return memory[addr]; // Return stored value for other registers
    }
}

// Write to I/O registers (0xFF00-0xFF7F)
void Emulator::writeIORegister(uint16_t addr, uint8_t value) {
    switch (addr) {
    case 0xFF46: { // DMA transfer register
        // DMA transfers 160 bytes from XX00-XX9F to OAM (FE00-FE9F)
        uint16_t source = value << 8;
        for (int i = 0; i < 160; i++) {
            uint8_t byte = readMemory(source + i);
            oam[i] = byte;
        }
        // DMA takes 671 cycles and blocks most memory access
        cpu.cycles += 671;
        memory[addr] = value;
        break;
    }
    case 0xFF04: // DIV register (writing resets it)
        memory[addr] = 0;
        break;
    case 0xFF41: // STAT register
        ppu.stat = (ppu.stat & 0x07) | (value & 0xF8); // Lower 3 bits read-only
        memory[addr] = ppu.stat;
        break;
    case 0xFF00: // Joypad register
        memory[addr] = (memory[addr] & 0x0F) | (value & 0xF0);
        break;
    case 0xFF05: // TIMA register (timer counter)
    case 0xFF06: // TMA register (timer modulo)
    case 0xFF07: // TAC register (timer control)
    case 0xFF0F: // IF register (interrupt flags)
        memory[addr] = value;
        break;
    case 0xFF40: // LCDC register
        ppu.lcdc = value;
        memory[addr] = value;
        break;
    case 0xFF42: // SCY register
        ppu.scy = value;
        memory[addr] = value;
        break;
    case 0xFF43: // SCX register
        ppu.scx = value;
        memory[addr] = value;
        break;
    case 0xFF44: // LY register (read-only, but games might try to write)
        // LY is read-only, ignore writes
        break;
    case 0xFF45: // LYC register
        ppu.lyc = value;
        memory[addr] = value;
        break;
    case 0xFF47: // BGP register
        ppu.bgp = value;
        memory[addr] = value;
        break;
    case 0xFF48: // OBP0 register
        ppu.obp0 = value;
        memory[addr] = value;
        break;
    case 0xFF49: // OBP1 register
        ppu.obp1 = value;
        memory[addr] = value;
        break;
    case 0xFF4A: // WY register
        ppu.wy = value;
        memory[addr] = value;
        break;
    case 0xFF4B: // WX register
        ppu.wx = value;
        memory[addr] = value;
        break;
    default:
        memory[addr] = value; // Store value for other registers
        break;
    }
}

// Arithmetic helper functions
void Emulator::addToA(uint8_t value, bool carry) {
    uint8_t carry_in = carry ? 1 : 0;
    uint16_t result = cpu.A + value + carry_in;

    // Half carry: check if carry from bit 3 to bit 4
    bool half_carry = ((cpu.A & 0x0F) + (value & 0x0F) + carry_in) > 0x0F;

    // Full carry: check if result > 255
    bool full_carry = result > 0xFF;

    cpu.A = result & 0xFF;

    setZeroFlag(cpu.A == 0);
    setSubtractFlag(false);
    setHalfCarryFlag(half_carry);
    setCarryFlag(full_carry);
}

void Emulator::subFromA(uint8_t value, bool carry) {
    uint8_t carry_in = carry ? 1 : 0;
    int result = (int)cpu.A - (int)value - (int)carry_in;

    // Half carry: borrow from bit 4 to bit 3
    bool half_carry = ((int)(cpu.A & 0x0F) - (int)(value & 0x0F) - (int)carry_in) < 0;

    // Full carry: result < 0
    bool full_carry = result < 0;

    cpu.A = result & 0xFF;

    setZeroFlag(cpu.A == 0);
    setSubtractFlag(true);
    setHalfCarryFlag(half_carry);
    setCarryFlag(full_carry);
}

void Emulator::compareA(uint8_t value) {
    int result = cpu.A - value;

    setZeroFlag(result == 0);
    setSubtractFlag(true);
    setHalfCarryFlag(((int)(cpu.A & 0x0F) - (int)(value & 0x0F)) < 0);
    setCarryFlag(result < 0);
}

void Emulator::andA(uint8_t value) {
    cpu.A &= value;
    setZeroFlag(cpu.A == 0);
    setSubtractFlag(false);
    setHalfCarryFlag(true);
    setCarryFlag(false);
}

void Emulator::orA(uint8_t value) {
    cpu.A |= value;
    setZeroFlag(cpu.A == 0);
    setSubtractFlag(false);
    setHalfCarryFlag(false);
    setCarryFlag(false);
}

void Emulator::xorA(uint8_t value) {
    cpu.A ^= value;
    setZeroFlag(cpu.A == 0);
    setSubtractFlag(false);
    setHalfCarryFlag(false);
    setCarryFlag(false);
}

// Stack operations
void Emulator::pushStackWord(uint16_t value) {
    cpu.SP -= 2;
    writeMemory(cpu.SP, value & 0xFF);     // Low byte
    writeMemory(cpu.SP + 1, value >> 8);   // High byte  
}

uint16_t Emulator::popStackWord() {
    uint16_t value = readMemory(cpu.SP) | (readMemory(cpu.SP + 1) << 8);
    cpu.SP += 2;
    return value;
}

// Joypad register handling
uint8_t Emulator::readJoypadRegister() const {
    uint8_t joyp = memory[0xFF00];

    if (!(joyp & 0x10)) { // Direction keys selected
        uint8_t directions = 0x0F;
        if (joypad.down)  directions &= ~0x08;
        if (joypad.up)    directions &= ~0x04;
        if (joypad.left)  directions &= ~0x02;
        if (joypad.right) directions &= ~0x01;
        return (joyp & 0xF0) | directions;
    }

    if (!(joyp & 0x20)) { // Button keys selected  
        uint8_t buttons = 0x0F;
        if (joypad.start)  buttons &= ~0x08;
        if (joypad.select) buttons &= ~0x04;
        if (joypad.b)      buttons &= ~0x02;
        if (joypad.a)      buttons &= ~0x01;
        return (joyp & 0xF0) | buttons;
    }

    return joyp;
}

// Enhanced CPU instruction execution with more complete instruction set
void Emulator::executeInstruction() {
    if (!running) return;

    // FETCH: Get the instruction byte from memory at PC
    uint8_t opcode = readMemory(cpu.PC);
    cpu.PC++;  // Increment Program Counter to next byte

    // Each instruction takes different amounts of time (cycles)
    // Real Game Boy CPU runs at ~4.19 MHz, so timing is important
    int cycles_taken = 1;

    // DECODE & EXECUTE: Figure out what instruction this is and do it
    switch (opcode) {
        // 0x00: NOP - No Operation (do nothing for 1 cycle)
    case 0x00:
        cycles_taken = 1;
        break;

        // 16-bit loads: LD rr, nn
    case 0x01: { // LD BC, nn
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        cpu.C = low; cpu.B = high;
        cycles_taken = 3;
        break;
    }
    case 0x11: { // LD DE, nn
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        cpu.E = low; cpu.D = high;
        cycles_taken = 3;
        break;
    }
    case 0x21: { // LD HL, nn
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        cpu.L = low; cpu.H = high;
        cycles_taken = 3;
        break;
    }
    case 0x31: { // LD SP, nn
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        cpu.SP = (high << 8) | low;
        cycles_taken = 3;
        break;
    }

             // 8-bit loads: LD r, n
    case 0x06: cpu.B = readMemory(cpu.PC++); cycles_taken = 2; break; // LD B, n
    case 0x0E: cpu.C = readMemory(cpu.PC++); cycles_taken = 2; break; // LD C, n
    case 0x16: cpu.D = readMemory(cpu.PC++); cycles_taken = 2; break; // LD D, n
    case 0x1E: cpu.E = readMemory(cpu.PC++); cycles_taken = 2; break; // LD E, n
    case 0x26: cpu.H = readMemory(cpu.PC++); cycles_taken = 2; break; // LD H, n
    case 0x2E: cpu.L = readMemory(cpu.PC++); cycles_taken = 2; break; // LD L, n
    case 0x3E: cpu.A = readMemory(cpu.PC++); cycles_taken = 2; break; // LD A, n

        // Memory loads and stores
    case 0x02: { // LD (BC), A
        uint16_t addr = (cpu.B << 8) | cpu.C;
        writeMemory(addr, cpu.A);
        cycles_taken = 2;
        break;
    }
    case 0x12: { // LD (DE), A
        uint16_t addr = (cpu.D << 8) | cpu.E;
        writeMemory(addr, cpu.A);
        cycles_taken = 2;
        break;
    }
    case 0x22: { // LD (HL+), A - Store A at HL, then increment HL
        uint16_t addr = (cpu.H << 8) | cpu.L;
        writeMemory(addr, cpu.A);
        addr++;
        cpu.H = (addr >> 8) & 0xFF;
        cpu.L = addr & 0xFF;
        cycles_taken = 2;
        break;
    }
    case 0x32: { // LD (HL-), A - Store A at HL, then decrement HL
        uint16_t addr = (cpu.H << 8) | cpu.L;
        writeMemory(addr, cpu.A);
        addr--;
        cpu.H = (addr >> 8) & 0xFF;
        cpu.L = addr & 0xFF;
        cycles_taken = 2;
        break;
    }
    case 0x0A: { // LD A, (BC)
        uint16_t addr = (cpu.B << 8) | cpu.C;
        cpu.A = readMemory(addr);
        cycles_taken = 2;
        break;
    }
    case 0x1A: { // LD A, (DE)
        uint16_t addr = (cpu.D << 8) | cpu.E;
        cpu.A = readMemory(addr);
        cycles_taken = 2;
        break;
    }
    case 0x2A: { // LD A, (HL+) - Load A from HL, then increment HL
        uint16_t addr = (cpu.H << 8) | cpu.L;
        cpu.A = readMemory(addr);
        addr++;
        cpu.H = (addr >> 8) & 0xFF;
        cpu.L = addr & 0xFF;
        cycles_taken = 2;
        break;
    }
    case 0x3A: { // LD A, (HL-) - Load A from HL, then decrement HL
        uint16_t addr = (cpu.H << 8) | cpu.L;
        cpu.A = readMemory(addr);
        addr--;
        cpu.H = (addr >> 8) & 0xFF;
        cpu.L = addr & 0xFF;
        cycles_taken = 2;
        break;
    }
    case 0x77: { // LD (HL), A
        uint16_t addr = (cpu.H << 8) | cpu.L;
        writeMemory(addr, cpu.A);
        cycles_taken = 2;
        break;
    }
    case 0x7E: { // LD A, (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        cpu.A = readMemory(addr);
        cycles_taken = 2;
        break;
    }

             // Register to register moves (LD r,r) - A complete set
    case 0x40: /* cpu.B = cpu.B; */ cycles_taken = 1; break; // LD B, B (NOP)
    case 0x41: cpu.B = cpu.C; cycles_taken = 1; break;       // LD B, C
    case 0x42: cpu.B = cpu.D; cycles_taken = 1; break;       // LD B, D
    case 0x43: cpu.B = cpu.E; cycles_taken = 1; break;       // LD B, E
    case 0x44: cpu.B = cpu.H; cycles_taken = 1; break;       // LD B, H
    case 0x45: cpu.B = cpu.L; cycles_taken = 1; break;       // LD B, L
    case 0x47: cpu.B = cpu.A; cycles_taken = 1; break;       // LD B, A

    case 0x48: cpu.C = cpu.B; cycles_taken = 1; break;       // LD C, B
    case 0x49: /* cpu.C = cpu.C; */ cycles_taken = 1; break; // LD C, C (NOP)
    case 0x4A: cpu.C = cpu.D; cycles_taken = 1; break;       // LD C, D
    case 0x4B: cpu.C = cpu.E; cycles_taken = 1; break;       // LD C, E
    case 0x4C: cpu.C = cpu.H; cycles_taken = 1; break;       // LD C, H
    case 0x4D: cpu.C = cpu.L; cycles_taken = 1; break;       // LD C, L
    case 0x4F: cpu.C = cpu.A; cycles_taken = 1; break;       // LD C, A

    case 0x50: cpu.D = cpu.B; cycles_taken = 1; break;       // LD D, B
    case 0x51: cpu.D = cpu.C; cycles_taken = 1; break;       // LD D, C
    case 0x52: /* cpu.D = cpu.D; */ cycles_taken = 1; break; // LD D, D (NOP)
    case 0x53: cpu.D = cpu.E; cycles_taken = 1; break;       // LD D, E
    case 0x54: cpu.D = cpu.H; cycles_taken = 1; break;       // LD D, H
    case 0x55: cpu.D = cpu.L; cycles_taken = 1; break;       // LD D, L
    case 0x57: cpu.D = cpu.A; cycles_taken = 1; break;       // LD D, A

    case 0x58: cpu.E = cpu.B; cycles_taken = 1; break;       // LD E, B
    case 0x59: cpu.E = cpu.C; cycles_taken = 1; break;       // LD E, C
    case 0x5A: cpu.E = cpu.D; cycles_taken = 1; break;       // LD E, D
    case 0x5B: /* cpu.E = cpu.E; */ cycles_taken = 1; break; // LD E, E (NOP)
    case 0x5C: cpu.E = cpu.H; cycles_taken = 1; break;       // LD E, H
    case 0x5D: cpu.E = cpu.L; cycles_taken = 1; break;       // LD E, L
    case 0x5F: cpu.E = cpu.A; cycles_taken = 1; break;       // LD E, A

    case 0x60: cpu.H = cpu.B; cycles_taken = 1; break;       // LD H, B
    case 0x61: cpu.H = cpu.C; cycles_taken = 1; break;       // LD H, C
    case 0x62: cpu.H = cpu.D; cycles_taken = 1; break;       // LD H, D
    case 0x63: cpu.H = cpu.E; cycles_taken = 1; break;       // LD H, E
    case 0x64: /* cpu.H = cpu.H; */ cycles_taken = 1; break; // LD H, H (NOP)
    case 0x65: cpu.H = cpu.L; cycles_taken = 1; break;       // LD H, L
    case 0x67: cpu.H = cpu.A; cycles_taken = 1; break;       // LD H, A

    case 0x68: cpu.L = cpu.B; cycles_taken = 1; break;       // LD L, B
    case 0x69: cpu.L = cpu.C; cycles_taken = 1; break;       // LD L, C
    case 0x6A: cpu.L = cpu.D; cycles_taken = 1; break;       // LD L, D
    case 0x6B: cpu.L = cpu.E; cycles_taken = 1; break;       // LD L, E
    case 0x6C: cpu.L = cpu.H; cycles_taken = 1; break;       // LD L, H
    case 0x6D: /* cpu.L = cpu.L; */ cycles_taken = 1; break; // LD L, L (NOP)
    case 0x6F: cpu.L = cpu.A; cycles_taken = 1; break;       // LD L, A

    case 0x78: cpu.A = cpu.B; cycles_taken = 1; break;       // LD A, B
    case 0x79: cpu.A = cpu.C; cycles_taken = 1; break;       // LD A, C
    case 0x7A: cpu.A = cpu.D; cycles_taken = 1; break;       // LD A, D
    case 0x7B: cpu.A = cpu.E; cycles_taken = 1; break;       // LD A, E
    case 0x7C: cpu.A = cpu.H; cycles_taken = 1; break;       // LD A, H
    case 0x7D: cpu.A = cpu.L; cycles_taken = 1; break;       // LD A, L
    case 0x7F: /* cpu.A = cpu.A; */ cycles_taken = 1; break; // LD A, A (NOP)
    
    // LD (HL), r instructions - Store register into memory at HL
    case 0x70: { // LD (HL), B
        uint16_t addr = (cpu.H << 8) | cpu.L;
        writeMemory(addr, cpu.B);
        cycles_taken = 2;
        break;
    }
    case 0x71: { // LD (HL), C
        uint16_t addr = (cpu.H << 8) | cpu.L;
        writeMemory(addr, cpu.C);
        cycles_taken = 2;
        break;
    }
    case 0x72: { // LD (HL), D
        uint16_t addr = (cpu.H << 8) | cpu.L;
        writeMemory(addr, cpu.D);
        cycles_taken = 2;
        break;
    }
    case 0x73: { // LD (HL), E
        uint16_t addr = (cpu.H << 8) | cpu.L;
        writeMemory(addr, cpu.E);
        cycles_taken = 2;
        break;
    }
    case 0x74: { // LD (HL), H
        uint16_t addr = (cpu.H << 8) | cpu.L;
        writeMemory(addr, cpu.H);
        cycles_taken = 2;
        break;
    }
    case 0x75: { // LD (HL), L
        uint16_t addr = (cpu.H << 8) | cpu.L;
        writeMemory(addr, cpu.L);
        cycles_taken = 2;
        break;
    }

    // LD r, (HL) instructions - Load from memory at HL into register
    case 0x46: { // LD B, (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        cpu.B = readMemory(addr);
        cycles_taken = 2;
        break;
    }
    case 0x4E: { // LD C, (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        cpu.C = readMemory(addr);
        cycles_taken = 2;
        break;
    }
    case 0x56: { // LD D, (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        cpu.D = readMemory(addr);
        cycles_taken = 2;
        break;
    }
    case 0x5E: { // LD E, (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        cpu.E = readMemory(addr);
        cycles_taken = 2;
        break;
    }
    case 0x66: { // LD H, (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        cpu.H = readMemory(addr);
        cycles_taken = 2;
        break;
    }
    case 0x6E: { // LD L, (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        cpu.L = readMemory(addr);
        cycles_taken = 2;
        break;
    }

    //Conditional calls
    case 0xC4: { // CALL NZ, nn
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        if (!getZeroFlag()) {
            pushStackWord(cpu.PC);
            cpu.PC = (high << 8) | low;
            cycles_taken = 6;
        }
        else {
            cycles_taken = 3;
        }
        break;
    }
    case 0xCC: { // CALL Z, nn
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        if (getZeroFlag()) {
            pushStackWord(cpu.PC);
            cpu.PC = (high << 8) | low;
            cycles_taken = 6;
        }
        else {
            cycles_taken = 3;
        }
        break;
    }
    case 0xD4: { // CALL NC, nn
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        if (!getCarryFlag()) {
            pushStackWord(cpu.PC);
            cpu.PC = (high << 8) | low;
            cycles_taken = 6;
        }
        else {
            cycles_taken = 3;
        }
        break;
    }
    case 0xDC: { // CALL C, nn
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        if (getCarryFlag()) {
            pushStackWord(cpu.PC);
            cpu.PC = (high << 8) | low;
            cycles_taken = 6;
        }
        else {
            cycles_taken = 3;
        }
        break;
    }

             // ADD SP, n instruction
    case 0xE8: { // ADD SP, n (signed 8-bit offset)
        int8_t offset = (int8_t)readMemory(cpu.PC++);
        uint16_t result = cpu.SP + offset;

        setZeroFlag(false);
        setSubtractFlag(false);
        setHalfCarryFlag(((cpu.SP & 0x0F) + (offset & 0x0F)) > 0x0F);
        setCarryFlag(((cpu.SP & 0xFF) + (offset & 0xFF)) > 0xFF);

        cpu.SP = result;
        cycles_taken = 4;
        break;
    }

        // Increment operations (8-bit)
    case 0x04: { // INC B
        cpu.B++;
        setZeroFlag(cpu.B == 0);
        setSubtractFlag(false);
        setHalfCarryFlag((cpu.B & 0x0F) == 0);
        cycles_taken = 1;
        break;
    }
    case 0x0C: { // INC C
        cpu.C++;
        setZeroFlag(cpu.C == 0);
        setSubtractFlag(false);
        setHalfCarryFlag((cpu.C & 0x0F) == 0);
        cycles_taken = 1;
        break;
    }
    case 0x14: { // INC D
        cpu.D++;
        setZeroFlag(cpu.D == 0);
        setSubtractFlag(false);
        setHalfCarryFlag((cpu.D & 0x0F) == 0);
        cycles_taken = 1;
        break;
    }
    case 0x1C: { // INC E
        cpu.E++;
        setZeroFlag(cpu.E == 0);
        setSubtractFlag(false);
        setHalfCarryFlag((cpu.E & 0x0F) == 0);
        cycles_taken = 1;
        break;
    }
    case 0x24: { // INC H
        cpu.H++;
        setZeroFlag(cpu.H == 0);
        setSubtractFlag(false);
        setHalfCarryFlag((cpu.H & 0x0F) == 0);
        cycles_taken = 1;
        break;
    }
    case 0x2C: { // INC L
        cpu.L++;
        setZeroFlag(cpu.L == 0);
        setSubtractFlag(false);
        setHalfCarryFlag((cpu.L & 0x0F) == 0);
        cycles_taken = 1;
        break;
    }
    case 0x3C: { // INC A
        cpu.A++;
        setZeroFlag(cpu.A == 0);
        setSubtractFlag(false);
        setHalfCarryFlag((cpu.A & 0x0F) == 0);
        cycles_taken = 1;
        break;
    }

             // Decrement operations (8-bit)
    case 0x05: { // DEC B
        cpu.B--;
        setZeroFlag(cpu.B == 0);
        setSubtractFlag(true);
        setHalfCarryFlag((cpu.B & 0x0F) == 0x0F);
        cycles_taken = 1;
        break;
    }
    case 0x0D: { // DEC C
        cpu.C--;
        setZeroFlag(cpu.C == 0);
        setSubtractFlag(true);
        setHalfCarryFlag((cpu.C & 0x0F) == 0x0F);
        cycles_taken = 1;
        break;
    }
    case 0x15: { // DEC D
        cpu.D--;
        setZeroFlag(cpu.D == 0);
        setSubtractFlag(true);
        setHalfCarryFlag((cpu.D & 0x0F) == 0x0F);
        cycles_taken = 1;
        break;
    }
    case 0x1D: { // DEC E
        cpu.E--;
        setZeroFlag(cpu.E == 0);
        setSubtractFlag(true);
        setHalfCarryFlag((cpu.E & 0x0F) == 0x0F);
        cycles_taken = 1;
        break;
    }
    case 0x25: { // DEC H
        cpu.H--;
        setZeroFlag(cpu.H == 0);
        setSubtractFlag(true);
        setHalfCarryFlag((cpu.H & 0x0F) == 0x0F);
        cycles_taken = 1;
        break;
    }
    case 0x2D: { // DEC L
        cpu.L--;
        setZeroFlag(cpu.L == 0);
        setSubtractFlag(true);
        setHalfCarryFlag((cpu.L & 0x0F) == 0x0F);
        cycles_taken = 1;
        break;
    }
    case 0x3D: { // DEC A
        cpu.A--;
        setZeroFlag(cpu.A == 0);
        setSubtractFlag(true);
        setHalfCarryFlag((cpu.A & 0x0F) == 0x0F);
        cycles_taken = 1;
        break;
    }

             // 16-bit increment/decrement
    case 0x03: { // INC BC - Takes 2 cycles, NOT 1
        uint16_t bc = (cpu.B << 8) | cpu.C;
        bc++;
        cpu.B = (bc >> 8) & 0xFF;
        cpu.C = bc & 0xFF;
        cycles_taken = 2; // This is critical for timing tests
        break;
    }

    case 0x13: { // INC DE - Takes 2 cycles
        uint16_t de = (cpu.D << 8) | cpu.E;
        de++;
        cpu.D = (de >> 8) & 0xFF;
        cpu.E = de & 0xFF;
        cycles_taken = 2;
        break;
    }

    case 0x23: { // INC HL - Takes 2 cycles
        uint16_t hl = (cpu.H << 8) | cpu.L;
        hl++;
        cpu.H = (hl >> 8) & 0xFF;
        cpu.L = hl & 0xFF;
        cycles_taken = 2;
        break;
    }

    case 0x33: { // INC SP - Takes 2 cycles
        cpu.SP++;
        cycles_taken = 2;
        break;
    }

             // Same for DEC instructions:
    case 0x0B: { // DEC BC
        uint16_t bc = (cpu.B << 8) | cpu.C;
        bc--;
        cpu.B = (bc >> 8) & 0xFF;
        cpu.C = bc & 0xFF;
        cycles_taken = 2;
        break;
    }

    case 0x1B: { // DEC DE
        uint16_t de = (cpu.D << 8) | cpu.E;
        de--;
        cpu.D = (de >> 8) & 0xFF;
        cpu.E = de & 0xFF;
        cycles_taken = 2;
        break;
    }

    case 0x2B: { // DEC HL
        uint16_t hl = (cpu.H << 8) | cpu.L;
        hl--;
        cpu.H = (hl >> 8) & 0xFF;
        cpu.L = hl & 0xFF;
        cycles_taken = 2;
        break;
    }

    case 0x3B: { // DEC SP
        cpu.SP--;
        cycles_taken = 2;
        break;
    }

             // Arithmetic operations with A register
    case 0x80: addToA(cpu.B); cycles_taken = 1; break;      // ADD A, B
    case 0x81: addToA(cpu.C); cycles_taken = 1; break;      // ADD A, C
    case 0x82: addToA(cpu.D); cycles_taken = 1; break;      // ADD A, D
    case 0x83: addToA(cpu.E); cycles_taken = 1; break;      // ADD A, E
    case 0x84: addToA(cpu.H); cycles_taken = 1; break;      // ADD A, H
    case 0x85: addToA(cpu.L); cycles_taken = 1; break;      // ADD A, L
    case 0x87: addToA(cpu.A); cycles_taken = 1; break;      // ADD A, A

    case 0x88: addToA(cpu.B, getCarryFlag()); cycles_taken = 1; break; // ADC A, B
    case 0x89: addToA(cpu.C, getCarryFlag()); cycles_taken = 1; break; // ADC A, C  
    case 0x8A: addToA(cpu.D, getCarryFlag()); cycles_taken = 1; break; // ADC A, D
    case 0x8B: addToA(cpu.E, getCarryFlag()); cycles_taken = 1; break; // ADC A, E
    case 0x8C: addToA(cpu.H, getCarryFlag()); cycles_taken = 1; break; // ADC A, H
    case 0x8D: addToA(cpu.L, getCarryFlag()); cycles_taken = 1; break; // ADC A, L
    case 0x8F: addToA(cpu.A, getCarryFlag()); cycles_taken = 1; break; // ADC A, A

    case 0x90: subFromA(cpu.B); cycles_taken = 1; break;     // SUB B
    case 0x91: subFromA(cpu.C); cycles_taken = 1; break;     // SUB C
    case 0x92: subFromA(cpu.D); cycles_taken = 1; break;     // SUB D
    case 0x93: subFromA(cpu.E); cycles_taken = 1; break;     // SUB E
    case 0x94: subFromA(cpu.H); cycles_taken = 1; break;     // SUB H
    case 0x95: subFromA(cpu.L); cycles_taken = 1; break;     // SUB L
    case 0x97: subFromA(cpu.A); cycles_taken = 1; break;     // SUB A

    // SBC A, r instructions (Subtract with Carry) 
    case 0x98: subFromA(cpu.B, getCarryFlag()); cycles_taken = 1; break; // SBC A, B
    case 0x99: subFromA(cpu.C, getCarryFlag()); cycles_taken = 1; break; // SBC A, C
    case 0x9A: subFromA(cpu.D, getCarryFlag()); cycles_taken = 1; break; // SBC A, D  
    case 0x9B: subFromA(cpu.E, getCarryFlag()); cycles_taken = 1; break; // SBC A, E
    case 0x9C: subFromA(cpu.H, getCarryFlag()); cycles_taken = 1; break; // SBC A, H
    case 0x9D: subFromA(cpu.L, getCarryFlag()); cycles_taken = 1; break; // SBC A, L
    case 0x9F: subFromA(cpu.A, getCarryFlag()); cycles_taken = 1; break; // SBC A, A

    case 0xA0: andA(cpu.B); cycles_taken = 1; break;         // AND B
    case 0xA1: andA(cpu.C); cycles_taken = 1; break;         // AND C
    case 0xA2: andA(cpu.D); cycles_taken = 1; break;         // AND D
    case 0xA3: andA(cpu.E); cycles_taken = 1; break;         // AND E
    case 0xA4: andA(cpu.H); cycles_taken = 1; break;         // AND H
    case 0xA5: andA(cpu.L); cycles_taken = 1; break;         // AND L
    case 0xA7: andA(cpu.A); cycles_taken = 1; break;         // AND A

    case 0xA8: xorA(cpu.B); cycles_taken = 1; break;         // XOR B
    case 0xA9: xorA(cpu.C); cycles_taken = 1; break;         // XOR C
    case 0xAA: xorA(cpu.D); cycles_taken = 1; break;         // XOR D
    case 0xAB: xorA(cpu.E); cycles_taken = 1; break;         // XOR E
    case 0xAC: xorA(cpu.H); cycles_taken = 1; break;         // XOR H
    case 0xAD: xorA(cpu.L); cycles_taken = 1; break;         // XOR L
    case 0xAF: xorA(cpu.A); cycles_taken = 1; break;         // XOR A

    case 0xB0: orA(cpu.B); cycles_taken = 1; break;          // OR B
    case 0xB1: orA(cpu.C); cycles_taken = 1; break;          // OR C
    case 0xB2: orA(cpu.D); cycles_taken = 1; break;          // OR D
    case 0xB3: orA(cpu.E); cycles_taken = 1; break;          // OR E
    case 0xB4: orA(cpu.H); cycles_taken = 1; break;          // OR H
    case 0xB5: orA(cpu.L); cycles_taken = 1; break;          // OR L
    case 0xB7: orA(cpu.A); cycles_taken = 1; break;          // OR A

    case 0xB8: compareA(cpu.B); cycles_taken = 1; break;     // CP B
    case 0xB9: compareA(cpu.C); cycles_taken = 1; break;     // CP C
    case 0xBA: compareA(cpu.D); cycles_taken = 1; break;     // CP D
    case 0xBB: compareA(cpu.E); cycles_taken = 1; break;     // CP E
    case 0xBC: compareA(cpu.H); cycles_taken = 1; break;     // CP H
    case 0xBD: compareA(cpu.L); cycles_taken = 1; break;     // CP L
    case 0xBF: compareA(cpu.A); cycles_taken = 1; break;     // CP A

    // Immediate arithmetic
    case 0xC6: { // ADD A, n
        uint8_t value = readMemory(cpu.PC++);
        addToA(value);
        cycles_taken = 2;
        break;
    }
    case 0xCE: { // ADC A, n
        uint8_t value = readMemory(cpu.PC++);
        addToA(value, getCarryFlag()); // Must use current carry flag!
        cycles_taken = 2;
        break;
    }
    case 0xD6: { // SUB n
        uint8_t value = readMemory(cpu.PC++);
        subFromA(value);
        cycles_taken = 2;
        break;
    }
    case 0xDE: { // SBC A, n  
        uint8_t value = readMemory(cpu.PC++);
        subFromA(value, getCarryFlag()); // Must use current carry flag!
        cycles_taken = 2;
        break;
    }
    case 0xE6: { // AND n
        uint8_t value = readMemory(cpu.PC++);
        andA(value);
        cycles_taken = 2;
        break;
    }
    case 0xEE: { // XOR n
        uint8_t value = readMemory(cpu.PC++);
        xorA(value);
        cycles_taken = 2;
        break;
    }
    case 0xF6: { // OR n
        uint8_t value = readMemory(cpu.PC++);
        orA(value);
        cycles_taken = 2;
        break;
    }
    case 0xFE: { // CP n
        uint8_t value = readMemory(cpu.PC++);
        compareA(value);
        cycles_taken = 2;
        break;
    }

    // Jump instructions
    case 0xC3: { // JP nn - Unconditional jump
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        cpu.PC = (high << 8) | low;
        cycles_taken = 4;
        break;
    }
    case 0xE9: { // JP (HL) - Jump to address in HL
        cpu.PC = (cpu.H << 8) | cpu.L;
        cycles_taken = 1;
        break;
    }

    // Conditional jumps
    case 0xC2: { // JP NZ, nn - Jump if not zero
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        if (!getZeroFlag()) {
            cpu.PC = (high << 8) | low;
            cycles_taken = 4;
        }
        else {
            cycles_taken = 3;
        }
        break;
    }
    case 0xCA: { // JP Z, nn - Jump if zero
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        if (getZeroFlag()) {
            cpu.PC = (high << 8) | low;
            cycles_taken = 4;
        }
        else {
            cycles_taken = 3;
        }
        break;
    }
    case 0xD2: { // JP NC, nn - Jump if no carry
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        if (!getCarryFlag()) {
            cpu.PC = (high << 8) | low;
            cycles_taken = 4;
        }
        else {
            cycles_taken = 3;
        }
        break;
    }
    case 0xDA: { // JP C, nn - Jump if carry
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        if (getCarryFlag()) {
            cpu.PC = (high << 8) | low;
            cycles_taken = 4;
        }
        else {
            cycles_taken = 3;
        }
        break;
    }

    // Relative jumps
    case 0x18: { // JR n - Relative jump
        int8_t offset = (int8_t)readMemory(cpu.PC++);
        cpu.PC += offset;
        cycles_taken = 3;
        break;
    }
    case 0x20: { // JR NZ, n - Relative jump if not zero
        int8_t offset = (int8_t)readMemory(cpu.PC++);
        if (!getZeroFlag()) {
            cpu.PC += offset;
            cycles_taken = 3;
        }
        else {
            cycles_taken = 2;
        }
        break;
    }
    case 0x28: { // JR Z, n - Relative jump if zero
        int8_t offset = (int8_t)readMemory(cpu.PC++);
        if (getZeroFlag()) {
            cpu.PC += offset;
            cycles_taken = 3;
        }
        else {
            cycles_taken = 2;
        }
        break;
    }
    case 0x30: { // JR NC, n - Relative jump if no carry
        int8_t offset = (int8_t)readMemory(cpu.PC++);
        if (!getCarryFlag()) {
            cpu.PC += offset;
            cycles_taken = 3;
        }
        else {
            cycles_taken = 2;
        }
        break;
    }
    case 0x38: { // JR C, n - Relative jump if carry
        int8_t offset = (int8_t)readMemory(cpu.PC++);
        if (getCarryFlag()) {
            cpu.PC += offset;
            cycles_taken = 3;
        }
        else {
            cycles_taken = 2;
        }
        break;
    }

    // Call and return instructions
    case 0xCD: { // CALL nn - Call subroutine
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        pushStackWord(cpu.PC);
        cpu.PC = (high << 8) | low;
        cycles_taken = 6;
        break;
    }
    case 0xC9: { // RET - Return from subroutine
        cpu.PC = popStackWord();
        cycles_taken = 4;
        break;
    }
    case 0xD0: { // RET NC
        if (!getCarryFlag()) {
            cpu.PC = popStackWord();
            cycles_taken = 5;
        } else {
            cycles_taken = 2;
        }
        break;
    }
    case 0xC0: { // RET NZ
        if (!getZeroFlag()) {
            cpu.PC = popStackWord();
            cycles_taken = 5;
        } else {
            cycles_taken = 2;
        }
        break;
    }
    case 0xC8: { // RET Z
        if (getZeroFlag()) {
            cpu.PC = popStackWord();
            cycles_taken = 5;
        } else {
            cycles_taken = 2;
        }
        break;
    }
    case 0xD8: { // RET C
        if (getCarryFlag()) {
            cpu.PC = popStackWord();
            cycles_taken = 5;
        } else {
            cycles_taken = 2;
        }
        break;
    }

    case 0xD9: { // RETI - Return from interrupt
        cpu.PC = popStackWord();
        cpu.ime = true; // Re-enable interrupts
        cycles_taken = 4;
        break;
    }

    case 0xC7: { // RST 00h
        pushStackWord(cpu.PC);
        cpu.PC = 0x0000;
        cycles_taken = 4;
        break;
    }
    case 0xCF: { // RST 08h
        pushStackWord(cpu.PC);
        cpu.PC = 0x0008;
        cycles_taken = 4;
        break;
    }
    case 0xD7: { // RST 10h
        pushStackWord(cpu.PC);
        cpu.PC = 0x0010;
        cycles_taken = 4;
        break;
    }
    case 0xDF: { // RST 18h
        pushStackWord(cpu.PC);
        cpu.PC = 0x0018;
        cycles_taken = 4;
        break;
    }
    case 0xE7: { // RST 20h
        pushStackWord(cpu.PC);
        cpu.PC = 0x0020;
        cycles_taken = 4;
        break;
    }
    case 0xEF: { // RST 28h
        pushStackWord(cpu.PC);
        cpu.PC = 0x0028;
        cycles_taken = 4;
        break;
    }
    case 0xF7: { // RST 30h
        pushStackWord(cpu.PC);
        cpu.PC = 0x0030;
        cycles_taken = 4;
        break;
    }
    case 0xFF: { // RST 38h
        pushStackWord(cpu.PC);
        cpu.PC = 0x0038;
        cycles_taken = 4;
        break;
    }

    // Stack operations
    case 0xC5: { // PUSH BC
        pushStackWord((cpu.B << 8) | cpu.C);
        cycles_taken = 4;
        break;
    }
    case 0xD5: { // PUSH DE
        pushStackWord((cpu.D << 8) | cpu.E);
        cycles_taken = 4;
        break;
    }
    case 0xE5: { // PUSH HL
        pushStackWord((cpu.H << 8) | cpu.L);
        cycles_taken = 4;
        break;
    }
    case 0xF5: { // PUSH AF
        pushStackWord((cpu.A << 8) | cpu.F);
        cycles_taken = 4;
        break;
    }

    case 0xC1: { // POP BC
        uint16_t value = popStackWord();
        cpu.C = value & 0xFF;
        cpu.B = (value >> 8) & 0xFF;
        cycles_taken = 3;
        break;
    }
    case 0xD1: { // POP DE
        uint16_t value = popStackWord();
        cpu.E = value & 0xFF;
        cpu.D = (value >> 8) & 0xFF;
        cycles_taken = 3;
        break;
    }
    case 0xE1: { // POP HL
        uint16_t value = popStackWord();
        cpu.L = value & 0xFF;
        cpu.H = (value >> 8) & 0xFF;
        cycles_taken = 3;
        break;
    }
    case 0xF1: { // POP AF
        uint16_t value = popStackWord();
        cpu.F = value & 0xF0; // Lower 4 bits of F are always 0
        cpu.A = (value >> 8) & 0xFF;
        cycles_taken = 3;
        break;
    }

    // Interrupt control
    case 0xF3: { // DI - Disable interrupts
        cpu.ime = false;
        cycles_taken = 1;
        break;
    }
    case 0xFB: { // EI - Enable interrupts
        cpu.ime = true;
        cycles_taken = 1;
        break;
    }

    // High memory loads
    case 0xE0: { // LD ($FF00+n), A - Load A into high memory
        uint8_t offset = readMemory(cpu.PC++);
        writeMemory(0xFF00 + offset, cpu.A);
        cycles_taken = 3;
        break;
    }
    case 0xF0: { // LD A, ($FF00+n) - Load from high memory into A
        uint8_t offset = readMemory(cpu.PC++);
        cpu.A = readMemory(0xFF00 + offset);
        cycles_taken = 3;
        break;
    }
    case 0xE2: { // LD ($FF00+C), A - Load A into high memory at C
        writeMemory(0xFF00 + cpu.C, cpu.A);
        cycles_taken = 2;
        break;
    }
    case 0xF2: { // LD A, ($FF00+C) - Load from high memory at C into A
        cpu.A = readMemory(0xFF00 + cpu.C);
        cycles_taken = 2;
        break;
    }

    // Absolute memory loads
    case 0xEA: { // LD (nn), A - Store A at absolute address
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        uint16_t addr = (high << 8) | low;
        writeMemory(addr, cpu.A);
        cycles_taken = 4;
        break;
    }
    case 0xFA: { // LD A, (nn) - Load from absolute address into A
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        uint16_t addr = (high << 8) | low;
        cpu.A = readMemory(addr);
        cycles_taken = 4;
        break;
    }

    // Memory operations with (HL)
    case 0x34: { // INC (HL) - Increment memory at HL
        uint16_t addr = (cpu.H << 8) | cpu.L;
        uint8_t value = readMemory(addr);
        value++;
        writeMemory(addr, value);
        setZeroFlag(value == 0);
        setSubtractFlag(false);
        setHalfCarryFlag((value & 0x0F) == 0);
        cycles_taken = 3;
        break;
    }
    case 0x35: { // DEC (HL) - Decrement memory at HL
        uint16_t addr = (cpu.H << 8) | cpu.L;
        uint8_t value = readMemory(addr);
        value--;
        writeMemory(addr, value);
        setZeroFlag(value == 0);
        setSubtractFlag(true);
        setHalfCarryFlag((value & 0x0F) == 0x0F);
        cycles_taken = 3;
        break;
    }
    case 0x36: { // LD (HL), n - Load immediate value into memory at HL
        uint16_t addr = (cpu.H << 8) | cpu.L;
        uint8_t value = readMemory(cpu.PC++);
        writeMemory(addr, value);
        cycles_taken = 3;
        break;
    }

    // Arithmetic with (HL)
    case 0x86: { // ADD A, (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        uint8_t value = readMemory(addr);
        addToA(value);
        cycles_taken = 2;
        break;
    }
    case 0x8E: { // ADC A, (HL) - Add with carry from memory at HL
        uint16_t addr = (cpu.H << 8) | cpu.L;
        uint8_t value = readMemory(addr);
        addToA(value, getCarryFlag()); // Use carry flag state
        cycles_taken = 2;
        break;
    }
    case 0x96: { // SUB (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        uint8_t value = readMemory(addr);
        subFromA(value);
        cycles_taken = 2;
        break;
    }
    case 0x9E: { // SBC A, (HL) - Subtract with carry from memory at HL
        uint16_t addr = (cpu.H << 8) | cpu.L;
        uint8_t value = readMemory(addr);
        subFromA(value, getCarryFlag()); // Use carry flag state
        cycles_taken = 2;
        break;
    }
    case 0xA6: { // AND (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        uint8_t value = readMemory(addr);
        andA(value);
        cycles_taken = 2;
        break;
    }
    case 0xAE: { // XOR (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        uint8_t value = readMemory(addr);
        xorA(value);
        cycles_taken = 2;
        break;
    }
    case 0xB6: { // OR (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        uint8_t value = readMemory(addr);
        orA(value);
        cycles_taken = 2;
        break;
    }
    case 0xBE: { // CP (HL)
        uint16_t addr = (cpu.H << 8) | cpu.L;
        uint8_t value = readMemory(addr);
        compareA(value);
        cycles_taken = 2;
        break;
    }

    // Special instructions
    case 0x27: { // DAA - Decimal Adjust Accumulator (for BCD arithmetic)
        // This is complex - simplified implementation
        uint8_t a = cpu.A;
        if (!getSubtractFlag()) {
            if (getCarryFlag() || a > 0x99) {
                a += 0x60;
                setCarryFlag(true);
            }
            if (getHalfCarryFlag() || (a & 0x0F) > 0x09) {
                a += 0x06;
            }
        }
        else {
            if (getCarryFlag()) a -= 0x60;
            if (getHalfCarryFlag()) a -= 0x06;
        }
        cpu.A = a;
        setZeroFlag(cpu.A == 0);
        setHalfCarryFlag(false);
        cycles_taken = 1;
        break;
    }
    case 0x2F: { // CPL - Complement A (flip all bits)
        cpu.A = ~cpu.A;
        setSubtractFlag(true);
        setHalfCarryFlag(true);
        cycles_taken = 1;
        break;
    }
    case 0x37: { // SCF - Set Carry Flag
        setSubtractFlag(false);
        setHalfCarryFlag(false);
        setCarryFlag(true);
        cycles_taken = 1;
        break;
    }
    case 0x3F: { // CCF - Complement Carry Flag
        setSubtractFlag(false);
        setHalfCarryFlag(false);
        setCarryFlag(!getCarryFlag());
        cycles_taken = 1;
        break;
    }

    // Rotate and shift instructions
    case 0x07: { // RLCA - Rotate A left circular
        uint8_t carry = (cpu.A >> 7) & 1;
        cpu.A = (cpu.A << 1) | carry;
        setZeroFlag(false);
        setSubtractFlag(false);
        setHalfCarryFlag(false);
        setCarryFlag(carry != 0);
        cycles_taken = 1;
        break;
    }
    case 0x0F: { // RRCA - Rotate A right circular
        uint8_t carry = cpu.A & 1;
        cpu.A = (cpu.A >> 1) | (carry << 7);
        setZeroFlag(false);
        setSubtractFlag(false);
        setHalfCarryFlag(false);
        setCarryFlag(carry != 0);
        cycles_taken = 1;
        break;
    }
    case 0x17: { // RLA - Rotate A left through carry
        uint8_t old_carry = getCarryFlag() ? 1 : 0;
        uint8_t new_carry = (cpu.A >> 7) & 1;
        cpu.A = (cpu.A << 1) | old_carry;
        setZeroFlag(false);
        setSubtractFlag(false);
        setHalfCarryFlag(false);
        setCarryFlag(new_carry != 0);
        cycles_taken = 1;
        break;
    }
    case 0x1F: { // RRA - Rotate A right through carry
        uint8_t old_carry = getCarryFlag() ? 1 : 0;
        uint8_t new_carry = cpu.A & 1;
        cpu.A = (cpu.A >> 1) | (old_carry << 7);
        setZeroFlag(false);
        setSubtractFlag(false);
        setHalfCarryFlag(false);
        setCarryFlag(new_carry != 0);
        cycles_taken = 1;
        break;
    }

             // 16-bit arithmetic
    case 0x09: { // ADD HL, BC
        uint16_t hl = (cpu.H << 8) | cpu.L;
        uint16_t bc = (cpu.B << 8) | cpu.C;
        uint32_t result = hl + bc;
        cpu.H = (result >> 8) & 0xFF;
        cpu.L = result & 0xFF;
        setSubtractFlag(false);
        setHalfCarryFlag(((hl & 0x0FFF) + (bc & 0x0FFF)) > 0x0FFF);
        setCarryFlag(result > 0xFFFF);
        cycles_taken = 2;
        break;
    }
    case 0x19: { // ADD HL, DE
        uint16_t hl = (cpu.H << 8) | cpu.L;
        uint16_t de = (cpu.D << 8) | cpu.E;
        uint32_t result = hl + de;
        cpu.H = (result >> 8) & 0xFF;
        cpu.L = result & 0xFF;
        setSubtractFlag(false);
        setHalfCarryFlag(((hl & 0x0FFF) + (de & 0x0FFF)) > 0x0FFF);
        setCarryFlag(result > 0xFFFF);
        cycles_taken = 2;
        break;
    }
    case 0x29: { // ADD HL, HL
        uint16_t hl = (cpu.H << 8) | cpu.L;
        uint32_t result = hl + hl;
        cpu.H = (result >> 8) & 0xFF;
        cpu.L = result & 0xFF;
        setSubtractFlag(false);
        setHalfCarryFlag(((hl & 0x0FFF) + (hl & 0x0FFF)) > 0x0FFF);
        setCarryFlag(result > 0xFFFF);
        cycles_taken = 2;
        break;
    }
    case 0x39: { // ADD HL, SP
        uint16_t hl = (cpu.H << 8) | cpu.L;
        uint32_t result = hl + cpu.SP;
        cpu.H = (result >> 8) & 0xFF;
        cpu.L = result & 0xFF;
        setSubtractFlag(false);
        setHalfCarryFlag(((hl & 0x0FFF) + (cpu.SP & 0x0FFF)) > 0x0FFF);
        setCarryFlag(result > 0xFFFF);
        cycles_taken = 2;
        break;
    }

             // Load HL from SP + offset
    case 0xF8: { // LD HL, SP+n
        int8_t offset = (int8_t)readMemory(cpu.PC++);
        uint16_t result = cpu.SP + offset;
        cpu.H = (result >> 8) & 0xFF;
        cpu.L = result & 0xFF;
        setZeroFlag(false);
        setSubtractFlag(false);
        setHalfCarryFlag(((cpu.SP & 0x0F) + (offset & 0x0F)) > 0x0F);
        setCarryFlag(((cpu.SP & 0xFF) + (offset & 0xFF)) > 0xFF);
        cycles_taken = 3;
        break;
    }

    // Load SP from HL
    case 0xF9: { // LD SP, HL
        cpu.SP = (cpu.H << 8) | cpu.L;
        cycles_taken = 2;
        break;
    }

    // 0x76: HALT - Stop CPU execution until an interrupt occurs
    case 0x76: { // HALT
        // HALT timing is critical for interrupt tests
        if (cpu.ime) {
            // Normal HALT - wait for interrupt
            // In real hardware, this stops the CPU until an interrupt
            cycles_taken = 1;
            // For testing, you might want to set a flag instead of stopping
            // running = false;
        }
        else {
            // HALT with interrupts disabled
            uint8_t pending = memory[0xFFFF] & memory[0xFF0F] & 0x1F;
            if (pending) {
                // HALT bug - next instruction executed twice
                // This is very complex to implement correctly
                cycles_taken = 1;
            }
            else {
                // Normal HALT with no pending interrupts
                cycles_taken = 1;
                // running = false;
            }
        }
        break;
    }

    case 0x08: { // LD (nn), SP - Store stack pointer at absolute address
        uint8_t low = readMemory(cpu.PC++);
        uint8_t high = readMemory(cpu.PC++);
        uint16_t addr = (high << 8) | low;
        writeMemory(addr, cpu.SP & 0xFF);      // Store low byte
        writeMemory(addr + 1, cpu.SP >> 8);    // Store high byte
        cycles_taken = 5;
        break;
    }

    //0x10 - STOP
    case 0x10: { // STOP - Halt CPU and LCD until button press
        // Read the next byte (which should be 0x00)
        cpu.PC++;
        // In a real Game Boy, this would stop the CPU completely
        // For emulation, we can treat it similar to HALT
        // Some games use this for power management
        cycles_taken = 1;
        break;
    }

    // CB prefix instructions (extended instruction set)
    case 0xCB: {
        uint8_t cb_opcode = readMemory(cpu.PC++);

        bool is_hl_operation = (cb_opcode & 0x07) == 0x06;
        cycles_taken = is_hl_operation ? 4 : 2;

        uint8_t* reg = nullptr;
        uint16_t hl_addr = (cpu.H << 8) | cpu.L;
        uint8_t value = 0;
        bool isHL = false;

        // Helper to get register pointer or HL
        auto get_reg = [&](int code) -> uint8_t* {
            switch (code) {
            case 0: return &cpu.B;
            case 1: return &cpu.C;
            case 2: return &cpu.D;
            case 3: return &cpu.E;
            case 4: return &cpu.H;
            case 5: return &cpu.L;
            case 6: isHL = true; return nullptr; // (HL)
            case 7: return &cpu.A;
            }
            return nullptr;
            };

        int op = (cb_opcode >> 3) & 0x07;
        int reg_code = cb_opcode & 0x07;
        reg = get_reg(reg_code);
        value = isHL ? readMemory(hl_addr) : *reg;

        switch (cb_opcode) {
            // Rotates and shifts
        case 0x00: // RLC B
        case 0x01: // RLC C
        case 0x02: // RLC D
        case 0x03: // RLC E
        case 0x04: // RLC H
        case 0x05: // RLC L
        case 0x06: // RLC (HL)
        case 0x07: // RLC A
        {
            uint8_t res = (value << 1) | (value >> 7);
            setZeroFlag(res == 0);
            setSubtractFlag(false);
            setHalfCarryFlag(false);
            setCarryFlag((value >> 7) & 1);
            if (isHL) writeMemory(hl_addr, res); else *reg = res;
            cycles_taken = isHL ? 4 : 2;
            break;
        }
        case 0x08: // RRC B
        case 0x09: // RRC C
        case 0x0A: // RRC D
        case 0x0B: // RRC E
        case 0x0C: // RRC H
        case 0x0D: // RRC L
        case 0x0E: // RRC (HL)
        case 0x0F: // RRC A
        {
            uint8_t res = (value >> 1) | (value << 7);
            setZeroFlag(res == 0);
            setSubtractFlag(false);
            setHalfCarryFlag(false);
            setCarryFlag(value & 1);
            if (isHL) writeMemory(hl_addr, res); else *reg = res;
            cycles_taken = isHL ? 4 : 2;
            break;
        }
        case 0x10: // RL B
        case 0x11: // RL C
        case 0x12: // RL D
        case 0x13: // RL E
        case 0x14: // RL H
        case 0x15: // RL L
        case 0x16: // RL (HL)
        case 0x17: // RL A
        {
            uint8_t old_carry = getCarryFlag() ? 1 : 0;
            uint8_t new_carry = (value >> 7) & 1;
            uint8_t res = (value << 1) | old_carry;
            setZeroFlag(res == 0);
            setSubtractFlag(false);
            setHalfCarryFlag(false);
            setCarryFlag(new_carry);
            if (isHL) writeMemory(hl_addr, res); else *reg = res;
            cycles_taken = isHL ? 4 : 2;
            break;
        }
        case 0x18: // RR B
        case 0x19: // RR C
        case 0x1A: // RR D
        case 0x1B: // RR E
        case 0x1C: // RR H
        case 0x1D: // RR L
        case 0x1E: // RR (HL)
        case 0x1F: // RR A
        {
            uint8_t old_carry = getCarryFlag() ? 1 : 0;
            uint8_t new_carry = value & 1;
            uint8_t res = (value >> 1) | (old_carry << 7);
            setZeroFlag(res == 0);
            setSubtractFlag(false);
            setHalfCarryFlag(false);
            setCarryFlag(new_carry);
            if (isHL) writeMemory(hl_addr, res); else *reg = res;
            cycles_taken = isHL ? 4 : 2;
            break;
        }
        case 0x20: // SLA B
        case 0x21: // SLA C
        case 0x22: // SLA D
        case 0x23: // SLA E
        case 0x24: // SLA H
        case 0x25: // SLA L
        case 0x26: // SLA (HL)
        case 0x27: // SLA A
        {
            setCarryFlag((value >> 7) & 1);
            uint8_t res = value << 1;
            setZeroFlag(res == 0);
            setSubtractFlag(false);
            setHalfCarryFlag(false);
            if (isHL) writeMemory(hl_addr, res); else *reg = res;
            cycles_taken = isHL ? 4 : 2;
            break;
        }
        case 0x28: // SRA B
        case 0x29: // SRA C
        case 0x2A: // SRA D
        case 0x2B: // SRA E
        case 0x2C: // SRA H
        case 0x2D: // SRA L
        case 0x2E: // SRA (HL)
        case 0x2F: // SRA A
        {
            setCarryFlag(value & 1);
            uint8_t res = (value >> 1) | (value & 0x80);
            setZeroFlag(res == 0);
            setSubtractFlag(false);
            setHalfCarryFlag(false);
            if (isHL) writeMemory(hl_addr, res); else *reg = res;
            cycles_taken = isHL ? 4 : 2;
            break;
        }
        case 0x30: // SWAP B
        case 0x31: // SWAP C
        case 0x32: // SWAP D
        case 0x33: // SWAP E
        case 0x34: // SWAP H
        case 0x35: // SWAP L
        case 0x36: // SWAP (HL)
        case 0x37: // SWAP A
        {
            uint8_t res = ((value & 0x0F) << 4) | ((value & 0xF0) >> 4);
            setZeroFlag(res == 0);
            setSubtractFlag(false);
            setHalfCarryFlag(false);
            setCarryFlag(false);
            if (isHL) writeMemory(hl_addr, res); else *reg = res;
            cycles_taken = isHL ? 4 : 2;
            break;
        }
        case 0x38: // SRL B
        case 0x39: // SRL C
        case 0x3A: // SRL D
        case 0x3B: // SRL E
        case 0x3C: // SRL H
        case 0x3D: // SRL L
        case 0x3E: // SRL (HL)
        case 0x3F: // SRL A
        {
            setCarryFlag(value & 1);
            uint8_t res = value >> 1;
            setZeroFlag(res == 0);
            setSubtractFlag(false);
            setHalfCarryFlag(false);
            if (isHL) writeMemory(hl_addr, res); else *reg = res;
            cycles_taken = isHL ? 4 : 2;
            break;
        }
        // BIT b, r
        default:
            if ((cb_opcode & 0xC0) == 0x40) {
                int bit = (cb_opcode >> 3) & 0x07;
                setZeroFlag(((value >> bit) & 1) == 0);
                setSubtractFlag(false);
                setHalfCarryFlag(true);
                cycles_taken = isHL ? 3 : 2;
            }
            // RES b, r
            else if ((cb_opcode & 0xC0) == 0x80) {
                int bit = (cb_opcode >> 3) & 0x07;
                uint8_t res = value & ~(1 << bit);
                if (isHL) writeMemory(hl_addr, res); else *reg = res;
                cycles_taken = isHL ? 4 : 2;
            }
            // SET b, r
            else if ((cb_opcode & 0xC0) == 0xC0) {
                int bit = (cb_opcode >> 3) & 0x07;
                uint8_t res = value | (1 << bit);
                if (isHL) writeMemory(hl_addr, res); else *reg = res;
                cycles_taken = isHL ? 4 : 2;
            }
            else {
                std::cout << "CB instruction not implemented: $CB$" << std::hex << (int)cb_opcode << std::endl;
                cycles_taken = 2;
            }
            break;
        }
        // CB-prefixed instructions implemented (bit ops, rotates, shifts, swap)
        break;
    }

    // Unknown instruction - this shouldn't happen with valid Game Boy code
    default:
        // Print debug info and stop emulation
        std::cout << "Unknown opcode: $" << std::hex << (int)opcode
            << " at PC: $" << (cpu.PC - 1) << std::endl;

        // For debugging - let's see what instruction this should be
            if (opcode >= 0x40 && opcode <= 0x7F) {
                std::cout << "This appears to be a LD instruction in the 0x40-0x7F range" << std::endl;
            }


        running = false;
        cycles_taken = 1;
        break;
    }

    // Add the cycles this instruction took to our total count
    cpu.cycles += cycles_taken;
}

// Create a simple test program to verify our CPU instructions work
void Emulator::loadTestProgram() {
    // This creates a small program that exercises our implemented instructions
    uint8_t test_program[] = {
        0x3E, 0x42,        // LD A, $42    - Load hex value 0x42 (66 decimal) into A
        0x06, 0x10,        // LD B, $10    - Load hex value 0x10 (16 decimal) into B  
        0x04,              // INC B        - B becomes 0x11 (17 decimal)
        0x04,              // INC B        - B becomes 0x12 (18 decimal)
        0x05,              // DEC B        - B becomes 0x11 (17 decimal) again
        0x80,              // ADD A, B     - A = 0x42 + 0x11 = 0x53
        0x01, 0x34, 0x12,  // LD BC, $1234 - Load 0x1234 into BC (B=0x12, C=0x34)
        0x03,              // INC BC       - BC becomes 0x1235 (B=0x12, C=0x35)
        0xAF,              // XOR A        - Clear A (A = 0)
        0x3C,              // INC A        - A = 1
        0x3C,              // INC A        - A = 2
        0x00,              // NOP          - Do nothing (pause for observation)
        0x76               // HALT         - Stop execution
    };

    // Copy our test program into Game Boy memory starting at address 0x0100
    // (This is where Game Boy programs normally start after boot ROM)
    for (size_t i = 0; i < sizeof(test_program); ++i) {
        memory[0x0100 + i] = test_program[i];
    }

    reset();      // Reset CPU to initial state
    running = true;  // Start execution

    std::cout << "Loaded test program - click 'Step' to watch it execute!" << std::endl;
}

//a test pattern to see if rendering works at all
void Emulator::loadTestPattern() {
    std::cout << "Loading test pattern into VRAM..." << std::endl;

    // Clear all VRAM first
    memset(vram, 0, sizeof(vram));

    // Create a more visible test pattern - a checkerboard tile
    uint8_t test_tile[16] = {
        0xFF, 0x00,  // ########
        0x00, 0xFF,  //         
        0xFF, 0x00,  // ########
        0x00, 0xFF,  //         
        0xFF, 0x00,  // ########
        0x00, 0xFF,  //         
        0xFF, 0x00,  // ########
        0x00, 0xFF   //         
    };

    // Put test tile at index 0
    for (int i = 0; i < 16; i++) {
        vram[i] = test_tile[i];
    }

    // Create a second tile - solid color
    for (int i = 16; i < 32; i += 2) {
        vram[i] = 0xFF;     // All pixels color 3
        vram[i + 1] = 0xFF;
    }

    // Fill tile map with alternating pattern
    for (int y = 0; y < 18; y++) {
        for (int x = 0; x < 20; x++) {
            int tile_id = ((x + y) % 2); // Alternate between tile 0 and 1
            vram[0x1800 + y * 32 + x] = tile_id;
        }
    }

    // Force LCD settings for visible output
    ppu.lcdc = 0x91; // LCD on, BG on, sprites off, BG map at 0x9800, BG data at 0x8000
    ppu.bgp = 0xE4;  // Palette: 11 10 01 00 (white, light gray, dark gray, black)
    ppu.scx = ppu.scy = 0; // No scrolling
    ppu.ly = 0;

    // Update memory-mapped registers
    memory[0xFF40] = ppu.lcdc;
    memory[0xFF47] = ppu.bgp;
    memory[0xFF42] = ppu.scy;
    memory[0xFF43] = ppu.scx;
    memory[0xFF44] = ppu.ly;

    // Force a screen update
    for (int line = 0; line < 144; line++) {
        renderScanline(line);
    }

    std::cout << "Test pattern loaded. You should see a checkerboard pattern!" << std::endl;
}

// Update the Game Boy's internal timers
void Emulator::updateTimers() {
    // More accurate timer implementation
    static uint16_t div_counter = 0;
    static uint16_t timer_counter = 0;

    // DIV register increments every 256 CPU cycles (16384 Hz)
    div_counter += cycles_since_last_update;
    while (div_counter >= 256) {
        div_counter -= 256;
        memory[0xFF04] = (memory[0xFF04] + 1) & 0xFF;
    }

    // TIMA timer - more precise implementation
    uint8_t tac = memory[0xFF07];
    if (tac & 0x04) { // Timer enabled
        // Timer frequencies in CPU cycles - EXACT values
        uint16_t timer_period;
        switch (tac & 0x03) {
        case 0: timer_period = 1024; break;  // 4096 Hz
        case 1: timer_period = 16; break;    // 262144 Hz  
        case 2: timer_period = 64; break;    // 65536 Hz
        case 3: timer_period = 256; break;   // 16384 Hz
        }

        timer_counter += cycles_since_last_update;

        while (timer_counter >= timer_period) {
            timer_counter -= timer_period;

            uint8_t tima = memory[0xFF05];
            if (tima == 0xFF) {
                // Timer overflow - critical timing here
                memory[0xFF05] = memory[0xFF06]; // Load TMA into TIMA
                memory[0xFF0F] |= 0x04; // Set timer interrupt flag

                // CRITICAL: Timer overflow takes exactly 1 cycle
                cpu.cycles += 1;
            }
            else {
                memory[0xFF05] = tima + 1;
            }
        }
    }
}

void Emulator::renderScanline(int line) {
    // LCD disabled: fill with white
    if (!(ppu.lcdc & 0x80)) {
        std::fill(&screen[line * 160], &screen[(line + 1) * 160], 0);
        return;
    }

    // Fill background (if enabled)
    if (ppu.lcdc & 0x01) {
        // Background tile map base: 0x9800 or 0x9C00 (in VRAM: 0x1800 or 0x1C00)
        uint16_t bg_map_base = (ppu.lcdc & 0x08) ? 0x1C00 : 0x1800;
        bool tile_data_select = (ppu.lcdc & 0x10); // true = 0x8000, false = 0x8800 (signed)

        for (int x = 0; x < 160; ++x) {
            // Scroll offsets
            uint8_t scrolled_x = (x + ppu.scx) & 0xFF;
            uint8_t scrolled_y = (line + ppu.scy) & 0xFF;

            int tile_x = scrolled_x / 8;
            int tile_y = scrolled_y / 8;
            uint16_t map_index = bg_map_base + tile_y * 32 + tile_x;

            // Bounds check for tile map
            if (map_index >= 0x2000) {
                screen[line * 160 + x] = 0;
                continue;
            }

            uint8_t tile_index = vram[map_index];

            uint16_t tile_addr;
            if (tile_data_select) {
                // 0x8000 method — unsigned tile index
                tile_addr = tile_index * 16;
            }
            else {
                // 0x8800 method — signed tile index
                // The 8800 method uses signed indexing where:
                // - Index 0 maps to address 0x9000 (VRAM offset 0x1000) 
                // - Index -1 maps to 0x8FF0, -2 to 0x8FE0, etc.
                // - Index +1 maps to 0x9010, +2 to 0x9020, etc.
                int8_t signed_index = static_cast<int8_t>(tile_index);
                tile_addr = 0x1000 + signed_index * 16;
            }

            int tile_line = scrolled_y % 8;
            uint16_t byte_addr = tile_addr + tile_line * 2;

            // More robust bounds check for tile data
            if (tile_addr >= 0x2000 || byte_addr >= 0x2000 || byte_addr + 1 >= 0x2000) {
                screen[line * 160 + x] = 0;
                continue;
            }

            // Additional check for negative tile_addr (can happen with large negative signed indices)
            if ((int16_t)tile_addr < 0) {
                screen[line * 160 + x] = 0;
                continue;
            }

            uint8_t lo = vram[byte_addr];
            uint8_t hi = vram[byte_addr + 1];

            int bit = 7 - (scrolled_x % 8);
            uint8_t color = ((hi >> bit) & 1) << 1 | ((lo >> bit) & 1);

            // Apply palette
            uint8_t palette_color = (ppu.bgp >> (color * 2)) & 0x03;
            screen[line * 160 + x] = palette_color;
        }
    }
    else {
        // BG disabled — clear line
        std::fill(&screen[line * 160], &screen[(line + 1) * 160], 0);
    }

    // Render sprites (if enabled)
    if (ppu.lcdc & 0x02) {
        int sprite_height = (ppu.lcdc & 0x04) ? 16 : 8;
        int sprites_drawn = 0;

        // Process sprites in reverse order for correct priority
        for (int i = 39; i >= 0 && sprites_drawn < 10; --i) {
            uint8_t y = oam[i * 4 + 0];
            uint8_t x = oam[i * 4 + 1];
            uint8_t tile = oam[i * 4 + 2];
            uint8_t attr = oam[i * 4 + 3];

            int sprite_y = y - 16;
            int sprite_x = x - 8;

            // Skip sprites not on this line
            if (line < sprite_y || line >= sprite_y + sprite_height)
                continue;

            // Skip sprites completely off-screen
            if (sprite_x <= -8 || sprite_x >= 160)
                continue;

            int tile_line = line - sprite_y;
            if (attr & 0x40) // Y flip
                tile_line = sprite_height - 1 - tile_line;

            if (sprite_height == 16) {
                tile &= 0xFE; // Only even tiles used in 8×16 mode
                if (tile_line >= 8) {
                    tile |= 0x01;
                    tile_line -= 8;
                }
            }

            uint16_t tile_addr = tile * 16 + tile_line * 2;
            if (tile_addr + 1 >= 0x2000) continue;

            uint8_t lo = vram[tile_addr];
            uint8_t hi = vram[tile_addr + 1];

            for (int px = 0; px < 8; ++px) {
                int screen_x = sprite_x + px;
                if (screen_x < 0 || screen_x >= 160) continue;

                int bit = (attr & 0x20) ? px : (7 - px); // X flip
                uint8_t color = ((hi >> bit) & 1) << 1 | ((lo >> bit) & 1);
                if (color == 0) continue; // Transparent

                // Priority: sprite behind BG color 1–3 if attr & 0x80
                if ((attr & 0x80) && (ppu.lcdc & 0x01) && screen[line * 160 + screen_x] != 0)
                    continue;

                uint8_t palette = (attr & 0x10) ? ppu.obp1 : ppu.obp0;
                uint8_t palette_color = (palette >> (color * 2)) & 0x03;
                screen[line * 160 + screen_x] = palette_color;
            }

            ++sprites_drawn;
        }
    }
}

void Emulator::updateGraphics() {
    // Handle LCD disable state
    if (!(ppu.lcdc & 0x80)) {
        ppu.ly = 0;
        ppu.stat = (ppu.stat & 0xFC); // Mode 0
        cycles_in_scanline = 0;
        memory[0xFF44] = 0;
        memory[0xFF41] = ppu.stat;
        return;
    }

    cycles_in_scanline += cycles_since_last_update;

    // CRITICAL: Handle LCD turn-on timing precisely
    static uint8_t prev_lcdc = 0;
    if ((ppu.lcdc & 0x80) && !(prev_lcdc & 0x80)) {
        // LCD just turned on - CRITICAL timing
        // When LCD turns on, it should start at the CURRENT position in the scanline
        // NOT reset to beginning. This is what the test is checking.

        // Keep current cycle position, don't reset to 0
        // ppu.ly stays at current value
        // cycles_in_scanline keeps current value

        // Set initial mode based on current timing
        if (ppu.ly >= 144) {
            ppu.stat = (ppu.stat & 0xFC) | 0x01; // VBlank
        }
        else if (cycles_in_scanline < 80) {
            ppu.stat = (ppu.stat & 0xFC) | 0x02; // OAM search
        }
        else if (cycles_in_scanline < 252) {
            ppu.stat = (ppu.stat & 0xFC) | 0x03; // Pixel transfer
        }
        else {
            ppu.stat = (ppu.stat & 0xFC) | 0x00; // HBlank
        }
    }
    prev_lcdc = ppu.lcdc;

    // More precise scanline timing
    if (ppu.ly < 144) { // Visible scanlines
        uint8_t old_mode = ppu.stat & 0x03;
        uint8_t new_mode = old_mode;

        if (cycles_in_scanline < 80) {
            new_mode = 0x02; // OAM Search
        }
        else if (cycles_in_scanline < 252) {
            new_mode = 0x03; // Pixel Transfer
        }
        else if (cycles_in_scanline < 456) {
            new_mode = 0x00; // HBlank
            // Render at start of HBlank
            if (old_mode != 0x00) {
                renderScanline(ppu.ly);
            }
        }

        // Update mode and trigger interrupts
        if (old_mode != new_mode) {
            ppu.stat = (ppu.stat & 0xFC) | new_mode;

            // Mode interrupts
            if (new_mode == 0x00 && (ppu.stat & 0x08)) { // HBlank interrupt
                memory[0xFF0F] |= 0x02;
            }
            if (new_mode == 0x02 && (ppu.stat & 0x20)) { // OAM interrupt
                memory[0xFF0F] |= 0x02;
            }
        }

        // End of scanline
        if (cycles_in_scanline >= 456) {
            cycles_in_scanline -= 456;
            ppu.ly++;

            // Start of VBlank
            if (ppu.ly == 144) {
                ppu.stat = (ppu.stat & 0xFC) | 0x01;
                ppu.vblank_flag = true;
                memory[0xFF0F] |= 0x01; // VBlank interrupt
                if (ppu.stat & 0x10) { // VBlank STAT interrupt
                    memory[0xFF0F] |= 0x02;
                }
            }
        }
    }
    else if (ppu.ly < 154) {
        // VBlank period
        ppu.stat = (ppu.stat & 0xFC) | 0x01;

        if (cycles_in_scanline >= 456) {
            cycles_in_scanline -= 456;
            ppu.ly++;

            if (ppu.ly >= 154) {
                ppu.ly = 0;
                // Don't change mode here - let normal logic handle it
            }
        }
    }

    // LYC comparison - CRITICAL timing
    if (ppu.ly == ppu.lyc) {
        if (!(ppu.stat & 0x04)) {
            ppu.stat |= 0x04;
            if (ppu.stat & 0x40) { // LYC interrupt
                memory[0xFF0F] |= 0x02;
            }
        }
    }
    else {
        ppu.stat &= ~0x04;
    }

    // Update registers
    memory[0xFF44] = ppu.ly;
    memory[0xFF41] = ppu.stat;
}

// Handle interrupt processing
void Emulator::handleInterrupts() {
    if (!cpu.ime) return;

    uint8_t ie = memory[0xFFFF];
    uint8_t if_reg = memory[0xFF0F];
    uint8_t triggered = ie & if_reg;

    if (triggered) {
        // Interrupt handling takes exactly 5 machine cycles
        cpu.ime = false; // Disable interrupts

        // Push PC in 2 cycles
        pushStackWord(cpu.PC);

        // Jump to interrupt vector (3 more cycles total)
        if (triggered & 0x01) { // VBlank (highest priority)
            cpu.PC = 0x0040;
            memory[0xFF0F] &= ~0x01;
        }
        else if (triggered & 0x02) { // LCD STAT
            cpu.PC = 0x0048;
            memory[0xFF0F] &= ~0x02;
        }
        else if (triggered & 0x04) { // Timer
            cpu.PC = 0x0050;
            memory[0xFF0F] &= ~0x04;
        }
        else if (triggered & 0x08) { // Serial
            cpu.PC = 0x0058;
            memory[0xFF0F] &= ~0x08;
        }
        else if (triggered & 0x10) { // Joypad
            cpu.PC = 0x0060;
            memory[0xFF0F] &= ~0x10;
        }

        cpu.cycles += 5; // Total interrupt latency
    }
}

void Emulator::debugTileData() {
    std::cout << "=== VRAM Tile Data Debug ===" << std::endl;
    std::cout << "First 16 bytes of tile 0:" << std::endl;
    for (int i = 0; i < 16; i++) {
        std::cout << std::hex << (int)vram[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Tile map data (first 32 bytes):" << std::endl;
    for (int i = 0; i < 32; i++) {
        std::cout << std::hex << (int)vram[0x1800 + i] << " "; // 0x9800 - 0x8000
    }
    std::cout << std::endl;

    std::cout << "PPU registers:" << std::endl;
    std::cout << "LCDC: " << std::hex << (int)ppu.lcdc << std::endl;
    std::cout << "BGP: " << std::hex << (int)ppu.bgp << std::endl;
    std::cout << "SCX: " << (int)ppu.scx << " SCY: " << (int)ppu.scy << std::endl;
}

void Emulator::debugRenderingState() {
    std::cout << "\n=== RENDERING DEBUG ===" << std::endl;

    // PPU state
    std::cout << "LCDC: $" << std::hex << (int)ppu.lcdc << std::endl;
    std::cout << "BGP:  $" << std::hex << (int)ppu.bgp << std::endl;
    std::cout << "SCX:  " << std::dec << (int)ppu.scx << " SCY: " << (int)ppu.scy << std::endl;
    std::cout << "LY:   " << (int)ppu.ly << std::endl;

    // Memory layout analysis
    bool bg_tile_data_select = ppu.lcdc & 0x10;
    bool bg_tile_map_select = ppu.lcdc & 0x08;

    std::cout << "BG Tile Data Method: " << (bg_tile_data_select ? "8000" : "8800") << std::endl;
    std::cout << "BG Tile Map: " << (bg_tile_map_select ? "9C00" : "9800") << std::endl;

    // Check first few tile map entries
    uint16_t map_base = bg_tile_map_select ? 0x1C00 : 0x1800;
    std::cout << "First tile map entries: ";
    for (int i = 0; i < 8; i++) {
        if (map_base + i < 0x2000) {
            std::cout << std::hex << (int)vram[map_base + i] << " ";
        }
    }
    std::cout << std::endl;

    // Check if we have any actual tile data
    bool has_tile_data = false;
    for (int i = 0; i < 0x1800 && !has_tile_data; i++) {
        if (vram[i] != 0) has_tile_data = true;
    }
    std::cout << "Has tile data: " << (has_tile_data ? "YES" : "NO") << std::endl;

    // Render one scanline and show what happened
    std::cout << "\nRendering scanline 0..." << std::endl;
    renderScanline(0);

    std::cout << "First 16 screen pixels: ";
    for (int i = 0; i < 16; i++) {
        std::cout << (int)screen[i] << " ";
    }
    std::cout << std::endl;
}

void Emulator::debugSpecificTile(int tile_index) {
    std::cout << "\n=== TILE " << tile_index << " DEBUG ===" << std::endl;

    uint16_t tile_addr = tile_index * 16;
    if (tile_addr + 15 >= 0x2000) {
        std::cout << "Invalid tile address!" << std::endl;
        return;
    }

    std::cout << "Tile address in VRAM: $" << std::hex << tile_addr << std::endl;
    std::cout << "Real address: $" << std::hex << (0x8000 + tile_addr) << std::endl;

    // Show raw hex data
    std::cout << "Raw data:" << std::endl;
    for (int i = 0; i < 16; i += 2) {
        uint8_t lo = vram[tile_addr + i];
        uint8_t hi = vram[tile_addr + i + 1];
        std::cout << "Line " << std::dec << (i / 2) << ": "
            << std::hex << (int)lo << " " << (int)hi << std::endl;
    }

    // Show visual representation
    std::cout << "Visual (0=white, 3=black):" << std::endl;
    for (int y = 0; y < 8; y++) {
        uint8_t lo = vram[tile_addr + y * 2];
        uint8_t hi = vram[tile_addr + y * 2 + 1];

        for (int x = 0; x < 8; x++) {
            int bit = 7 - x;
            uint8_t color = ((hi >> bit) & 1) << 1 | ((lo >> bit) & 1);
            std::cout << (int)color;
        }
        std::cout << std::endl;
    }

    // Check if this tile is used in the tile map
    uint16_t map_base = (ppu.lcdc & 0x08) ? 0x1C00 : 0x1800;
    int usage_count = 0;
    for (int i = 0; i < 1024; i++) {
        if (map_base + i < 0x2000 && vram[map_base + i] == tile_index) {
            usage_count++;
        }
    }
    std::cout << "Used in tile map: " << usage_count << " times" << std::endl;
}

void Emulator::validateVRAMData() {
    std::cout << "\n=== VRAM VALIDATION ===" << std::endl;

    // Check for corruption patterns
    int zero_bytes = 0, ff_bytes = 0, pattern_bytes = 0;

    for (int i = 0; i < 0x2000; i++) {
        uint8_t byte = vram[i];
        if (byte == 0x00) zero_bytes++;
        else if (byte == 0xFF) ff_bytes++;
        else if (byte == 0xAA || byte == 0x55) pattern_bytes++;
    }

    std::cout << "VRAM Statistics:" << std::endl;
    std::cout << "Zero bytes: " << zero_bytes << "/8192" << std::endl;
    std::cout << "FF bytes: " << ff_bytes << "/8192" << std::endl;
    std::cout << "Pattern bytes (AA/55): " << pattern_bytes << "/8192" << std::endl;

    // Check for valid tile data patterns
    int valid_tiles = 0;
    for (int tile = 0; tile < 384; tile++) {
        uint16_t addr = tile * 16;
        if (addr + 15 >= 0x2000) break;

        bool has_data = false;
        bool looks_valid = true;

        for (int i = 0; i < 16; i++) {
            uint8_t byte = vram[addr + i];
            if (byte != 0) has_data = true;

            // Check for obvious corruption (same byte repeated)
            if (i > 0 && byte == vram[addr + i - 1] && byte != 0 && byte != 0xFF) {
                // This might be okay, don't mark as invalid yet
            }
        }

        if (has_data && looks_valid) valid_tiles++;
    }

    std::cout << "Tiles with data: " << valid_tiles << "/384" << std::endl;

    // Check ROM header if available
    if (rom.size() > 0x150) {
        std::cout << "\nROM Header Info:" << std::endl;
        std::cout << "Cartridge type: $" << std::hex << (int)rom[0x147] << std::endl;
        std::cout << "ROM size: $" << std::hex << (int)rom[0x148] << std::endl;
        std::cout << "RAM size: $" << std::hex << (int)rom[0x149] << std::endl;

        // Show title
        std::cout << "Title: ";
        for (int i = 0x134; i < 0x144; i++) {
            char c = rom[i];
            if (c >= 32 && c < 127) std::cout << c;
            else std::cout << ".";
        }
        std::cout << std::endl;
    }
}

void Emulator::debug8800MethodTiles() {
    std::cout << "\n=== 8800 METHOD TILE DEBUG ===" << std::endl;

    // In 8800 method, tile index 0 points to VRAM address 0x1000
    // Let's check what's actually there

    std::cout << "Checking VRAM at 0x1000 (where tile index 0 should point):" << std::endl;

    // Check tile at 0x1000 (this is where tile index 0 points in 8800 method)
    uint16_t tile_addr = 0x1000;
    std::cout << "Tile data at VRAM 0x1000:" << std::endl;
    bool has_data = false;
    for (int i = 0; i < 16; i += 2) {
        uint8_t lo = vram[tile_addr + i];
        uint8_t hi = vram[tile_addr + i + 1];
        if (lo != 0 || hi != 0) has_data = true;
        std::cout << "Line " << (i / 2) << ": " << std::hex << (int)lo << " " << (int)hi << std::endl;
    }

    if (!has_data) {
        std::cout << "No data found at 0x1000! Checking nearby addresses..." << std::endl;

        // Check a few tiles around 0x1000
        for (int tile_offset = -2; tile_offset <= 2; tile_offset++) {
            uint16_t check_addr = 0x1000 + (tile_offset * 16);
            if (check_addr >= 0x2000) continue;

            bool tile_has_data = false;
            for (int i = 0; i < 16; i++) {
                if (vram[check_addr + i] != 0) {
                    tile_has_data = true;
                    break;
                }
            }

            if (tile_has_data) {
                std::cout << "Found data at VRAM offset: 0x" << std::hex << check_addr
                    << " (real addr: 0x" << (0x8000 + check_addr) << ")" << std::endl;
            }
        }
    }

    // Also check what the tile map is actually pointing to
    std::cout << "\nTile map analysis:" << std::endl;
    uint16_t map_base = (ppu.lcdc & 0x08) ? 0x1C00 : 0x1800;

    // Count different tile indices used
    std::map<uint8_t, int> tile_usage;
    for (int i = 0; i < 32 * 32; i++) {
        if (map_base + i < 0x2000) {
            uint8_t tile_idx = vram[map_base + i];
            tile_usage[tile_idx]++;
        }
    }

    std::cout << "Tile indices used in map:" << std::endl;
    for (auto& pair : tile_usage) {
        uint8_t tile_idx = pair.first;
        int count = pair.second;

        // Calculate where this tile index points in 8800 method
        int8_t signed_idx = static_cast<int8_t>(tile_idx);
        uint16_t addr = 0x1000 + (signed_idx * 16);

        std::cout << "Tile index " << (int)tile_idx
            << " (signed: " << (int)signed_idx << ")"
            << " -> VRAM 0x" << std::hex << addr
            << " used " << std::dec << count << " times" << std::endl;

        // Check if this tile has data
        if (addr < 0x2000) {
            bool has_tile_data = false;
            for (int i = 0; i < 16; i++) {
                if (vram[addr + i] != 0) {
                    has_tile_data = true;
                    break;
                }
            }
            if (has_tile_data) {
                std::cout << "  -> HAS DATA!" << std::endl;
            }
        }
    }
}