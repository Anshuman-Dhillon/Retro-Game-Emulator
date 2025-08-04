# Retro Game Emulator

A simple emulator for retro games, written in C++ in Visual Studio, inspired by the original Game Boy.
**This is an educational project**, intended to deepen understanding of low-level systems, CPU architecture, graphics pipelines, and hardware emulation.

---

## About the Project

**Retro Game Emulator** is a Game Boy emulator built for learning purposes. It simulates how the original handheld console operates in terms of instruction decoding, memory-mapped I/O, timers, and graphics rendering.

This project is not meant to be the most accurate emulator out there, but instead aims to explore how real hardware can be modeled in software from the ground up.

---

## Technical Features

- LR35902 (8-bit) CPU emulation
- Accurate instruction decoding and execution (passes Blargg’s `cpu_instrs.gb`)
- Memory management (ROM, RAM, VRAM, OAM, HRAM, I/O registers)
- PPU (Pixel Processing Unit) for scanline-based rendering
- VBlank, HBlank, and LCD STAT interrupt handling
- Tile-based background and sprite rendering
- Supports `.gb` ROMs (games or test programs)
- ImGui-powered interface

---

## How It Works

### CPU Emulation

The emulator models the Game Boy’s **LR35902 processor**, a mix between the Intel 8080 and Z80 architectures. Instructions are fetched from memory using the **Program Counter (PC)**, decoded, and executed cycle-by-cycle.

- CPU registers (A, B, C, D, E, H, L, F) are emulated
- Flags (Zero, Subtract, Half-Carry, Carry) are handled per instruction
- Stack operations (`PUSH`, `POP`, `CALL`, `RET`) are fully supported
- Implements a basic interrupt system (VBlank, Timer, etc.)

### Memory Map

The emulator simulates the Game Boy’s 64KB address space:

| Address Range | Description                      |
|---------------|----------------------------------|
| `0000–3FFF`   | ROM Bank 0 (fixed)               |
| `4000–7FFF`   | Switchable ROM Bank (not yet implemented) |
| `8000–9FFF`   | Video RAM (tile & background data) |
| `A000–BFFF`   | External RAM (for save data)     |
| `C000–DFFF`   | Work RAM                         |
| `FE00–FE9F`   | Sprite Attribute Table (OAM)     |
| `FF00–FF7F`   | I/O Registers (timers, LCD, etc.)|
| `FF80–FFFE`   | High RAM (HRAM)                  |
| `FFFF`        | Interrupt Enable Register        |

### PPU (Graphics Rendering)

The PPU draws the screen line by line (scanlines), in a cycle-accurate manner:

- 160×144 pixel resolution
- Tile-based background rendering
- 8x8 or 8x16 sprites with palette and flipping
- Rendering phases:
  - Mode 2: OAM Search (80 cycles)
  - Mode 3: Pixel Transfer (172 cycles)
  - Mode 0: HBlank (204 cycles)
  - Mode 1: VBlank (lines 144–153)

---

## Interface & Debug Tools

This emulator includes optional debugging tools via **Dear ImGui**:

- Memory viewer
- CPU register window
- Tile viewer
- Scanline PPU mode visualizer
- Interrupt flags and timers
- Step-by-step execution (CPU and scanline stepping)

---

### Requirements and Dependencies

- Visual Studio (2022+)
- C++17 or newer
- GLFW
- OpenGL 3.3+
- Dear ImGui

### Build & Run

1. Clone the repository
2. Open the `.sln` file in Visual Studio
3. Build and run
4. Click on file > Load ROM to load a `.gb` file and start emulation

---

## ⚠️ Limitations

- No sound (APU not implemented)
- Interrupt timing may be slightly off (not cycle-perfect yet)
- Some homebrew timing test ROMs (like `oam_bug.gb`) may fail
- Only `.gb` (non-color) ROMs supported right now

---

## Learning Goals

This project was built to:

- Understand how classic consoles work internally
- Learn CPU and memory emulation
- Understand graphics pipelines at the hardware level
- Practice debugging with an immediate mode library
- Explore low-level I/O, timers, and interrupt logic

---

## Acknowledgments/Resources Used

- [GBDev Wiki](https://gbdev.io/)
- [Blargg’s test ROMs](https://github.com/retrio/gb-test-roms)
- [ImGui](https://github.com/ocornut/imgui)
- [Dear ImGui + GLFW examples](https://github.com/ocornut/imgui/wiki)
- [The Ultimate Game Boy Talk (33c3) - YouTube](https://youtu.be/HyzD8pNlpwI)
- [Pandocs](https://gbdev.io/pandocs)
---

## License

This is a personal learning project and currently not under a specific license.  
You're free to explore and experiment with the source code for educational purposes.
