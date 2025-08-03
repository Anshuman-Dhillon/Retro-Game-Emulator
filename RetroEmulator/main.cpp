// main.cpp - Game Boy Emulator with Dear ImGui Debug Interface
// This file creates the main window and debug GUI for the emulator

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
// #include "imgui_memory_editor.h" // Optional third-party component
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <algorithm>
#include "emulator.h"
#include "tinyfiledialogs/tinyfiledialogs.h"
#include <deque>
#include <vector>

// Forward declare the emulator instance
Emulator* g_emulator = nullptr;

// GLFW error callback - prints OpenGL/window errors to console
static void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

// Convert Game Boy screen data to OpenGL texture for display
// Game Boy screen: 160x144 pixels, 2 bits per pixel (4 shades of gray)
// OpenGL needs: RGB format, 8 bits per channel
GLuint createScreenTexture(const uint8_t* screen_data) {
    static GLuint texture = 0;  // Reuse same texture ID
    if (texture == 0) {
        glGenTextures(1, &texture);  // Create texture first time
    }

    // Convert 2-bit grayscale to 24-bit RGB
    static uint8_t rgb_data[160 * 144 * 3];  // RGB buffer

    // Game Boy color palette (darkest to lightest)
    const uint8_t colors[4][3] = {
        {255, 255, 255}, // 0 = White (lightest)
        {170, 170, 170}, // 1 = Light gray  
        {85, 85, 85},    // 2 = Dark gray
        {0, 0, 0}        // 3 = Black (darkest)
    };

    // Convert each Game Boy pixel to RGB
    for (int i = 0; i < 160 * 144; ++i) {
        uint8_t shade = screen_data[i] & 0x3;  // Get 2-bit shade value
        rgb_data[i * 3 + 0] = colors[shade][0]; // Red
        rgb_data[i * 3 + 1] = colors[shade][1]; // Green  
        rgb_data[i * 3 + 2] = colors[shade][2]; // Blue
    }

    // Upload to OpenGL texture
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 160, 144, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb_data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); // Pixelated scaling
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // Sharp pixels

    return texture;
}

// Key callback function
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (!g_emulator) return;

    bool pressed = (action == GLFW_PRESS || action == GLFW_REPEAT);

    switch (key) {
    case GLFW_KEY_UP:    g_emulator->joypad.up = pressed; break;
    case GLFW_KEY_DOWN:  g_emulator->joypad.down = pressed; break;
    case GLFW_KEY_LEFT:  g_emulator->joypad.left = pressed; break;
    case GLFW_KEY_RIGHT: g_emulator->joypad.right = pressed; break;
    case GLFW_KEY_Z:     g_emulator->joypad.a = pressed; break;      // A button
    case GLFW_KEY_X:     g_emulator->joypad.b = pressed; break;      // B button
    case GLFW_KEY_ENTER: g_emulator->joypad.start = pressed; break;  // Start
    case GLFW_KEY_SPACE: g_emulator->joypad.select = pressed; break; // Select
    }
}

static std::vector<float> timing_history(200, 0.0f);
static std::vector<uint8_t> ly_history(456, 0);
static std::vector<uint8_t> stat_history(456, 0);
static std::vector<uint8_t> interrupt_history(1000, 0);
static bool timing_paused = false;
static int breakpoint_scanline = -1;
static int breakpoint_cycle = -1;
static bool step_mode = false;
static int step_count = 1;

// 1. Real-time PPU Timing Visualizer
void DrawPPUTimingWindow(Emulator& emu) {
    if (ImGui::Begin("PPU Timing Analyzer")) {

        // Current PPU state display
        ImGui::Text("PPU State:");
        ImGui::Text("LY: %d | Cycles in scanline: %d", emu.ppu.ly, emu.cycles_in_scanline);
        ImGui::Text("LCDC: $%02X | STAT: $%02X", emu.ppu.lcdc, emu.ppu.stat);

        uint8_t mode = emu.ppu.stat & 0x03;
        const char* mode_names[] = { "HBlank", "VBlank", "OAM", "PixelTransfer" };
        ImGui::Text("Mode: %d (%s)", mode, mode_names[mode]);

        // LCD enable/disable controls
        bool lcd_enabled = emu.ppu.lcdc & 0x80;
        if (ImGui::Checkbox("LCD Enabled", &lcd_enabled)) {
            if (lcd_enabled) {
                emu.ppu.lcdc |= 0x80;
            }
            else {
                emu.ppu.lcdc &= ~0x80;
            }
            emu.memory[0xFF40] = emu.ppu.lcdc;
        }

        ImGui::Separator();

        // Scanline timing visualization
        ImGui::Text("Scanline Timing Breakdown:");

        // Visual timing bar
        float progress = (float)emu.cycles_in_scanline / 456.0f;
        ImGui::ProgressBar(progress, ImVec2(-1, 20), "");

        // Mode timing markers
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
        ImVec2 canvas_size = ImVec2(400, 30);

        // Background
        draw_list->AddRectFilled(canvas_pos,
            ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
            IM_COL32(50, 50, 50, 255));

        // Mode regions
        float oam_end = 80.0f / 456.0f * canvas_size.x;
        float pixel_end = 252.0f / 456.0f * canvas_size.x;

        // OAM region (red)
        draw_list->AddRectFilled(canvas_pos,
            ImVec2(canvas_pos.x + oam_end, canvas_pos.y + canvas_size.y),
            IM_COL32(255, 100, 100, 200));

        // Pixel Transfer region (blue)
        draw_list->AddRectFilled(ImVec2(canvas_pos.x + oam_end, canvas_pos.y),
            ImVec2(canvas_pos.x + pixel_end, canvas_pos.y + canvas_size.y),
            IM_COL32(100, 100, 255, 200));

        // HBlank region (green)
        draw_list->AddRectFilled(ImVec2(canvas_pos.x + pixel_end, canvas_pos.y),
            ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
            IM_COL32(100, 255, 100, 200));

        // Current position marker
        float current_pos = progress * canvas_size.x;
        draw_list->AddLine(ImVec2(canvas_pos.x + current_pos, canvas_pos.y),
            ImVec2(canvas_pos.x + current_pos, canvas_pos.y + canvas_size.y),
            IM_COL32(255, 255, 0, 255), 3.0f);

        ImGui::Dummy(canvas_size);

        // Timing labels
        ImGui::Text("OAM(0-79) | PixelTransfer(80-251) | HBlank(252-455)");

        ImGui::Separator();

        // LY history graph
        if (!timing_paused) {
            ly_history.erase(ly_history.begin());
            ly_history.push_back(emu.ppu.ly);
            stat_history.erase(stat_history.begin());
            stat_history.push_back(emu.ppu.stat & 0x03);
        }

        ImGui::Text("LY History:");
        {
            static std::vector<float> ly_history_f(ly_history.size());
            std::transform(ly_history.begin(), ly_history.end(), ly_history_f.begin(),
                [](uint8_t v) { return static_cast<float>(v); });
            ImGui::PlotLines("##LY", ly_history_f.data(), ly_history_f.size(), 0, nullptr, 0, 153, ImVec2(400, 60));
        }

        ImGui::Text("STAT Mode History:");
        {
            static std::vector<float> stat_history_f(stat_history.size());
            std::transform(stat_history.begin(), stat_history.end(), stat_history_f.begin(),
                [](uint8_t v) { return static_cast<float>(v); });
            ImGui::PlotLines("##STAT", stat_history_f.data(), stat_history_f.size(), 0, nullptr, 0, 3, ImVec2(400, 60));
        }

        if (ImGui::Button(timing_paused ? "Resume" : "Pause")) {
            timing_paused = !timing_paused;
        }
    }
    ImGui::End();
}

// 2. Timer Analysis Window
void DrawTimerAnalysisWindow(Emulator& emu) {
    if (ImGui::Begin("Timer Analysis")) { 

        ImGui::Text("Timer Registers:");
        ImGui::Text("DIV:  $%02X", emu.memory[0xFF04]);
        ImGui::Text("TIMA: $%02X", emu.memory[0xFF05]);
        ImGui::Text("TMA:  $%02X", emu.memory[0xFF06]);
        ImGui::Text("TAC:  $%02X", emu.memory[0xFF07]);

        uint8_t tac = emu.memory[0xFF07];
        bool timer_enabled = tac & 0x04;
        int timer_freq = tac & 0x03;

        ImGui::Text("Timer Enabled: %s", timer_enabled ? "YES" : "NO");

        const char* freq_names[] = { "4096 Hz", "262144 Hz", "65536 Hz", "16384 Hz" };
        ImGui::Text("Timer Frequency: %s", freq_names[timer_freq]);

        ImGui::Separator();

        // Timer frequency tester
        ImGui::Text("Timer Frequency Tester:");
        static uint8_t last_div = 0;
        static uint8_t last_tima = 0;
        static int div_increments = 0;
        static int tima_increments = 0;
        static int frame_counter = 0;

        uint8_t current_div = emu.memory[0xFF04];
        uint8_t current_tima = emu.memory[0xFF05];

        if (current_div != last_div) {
            div_increments++;
            last_div = current_div;
        }

        if (current_tima != last_tima) {
            tima_increments++;
            last_tima = current_tima;
        }

        frame_counter++;
        if (frame_counter >= 60) { // Reset every second (assuming 60 FPS)
            ImGui::Text("DIV increments/sec: %d (should be ~16384)", div_increments);
            ImGui::Text("TIMA increments/sec: %d", tima_increments);

            if (ImGui::Button("Reset Counters")) {
                div_increments = 0;
                tima_increments = 0;
                frame_counter = 0;
            }
        }

        // Timer control
        ImGui::Separator();
        ImGui::Text("Timer Control:");

        int new_tac = tac;
        if (ImGui::CheckboxFlags("Timer Enable", &new_tac, 0x04)) {
            emu.memory[0xFF07] = new_tac;
        }

        int freq_selection = tac & 0x03;
        if (ImGui::Combo("Frequency", &freq_selection, freq_names, 4)) {
            emu.memory[0xFF07] = (tac & 0xFC) | freq_selection;
        }
    }
    ImGui::End();
}

// 3. Interrupt Monitor
void DrawInterruptMonitor(Emulator& emu) {
    if (ImGui::Begin("Interrupt Monitor")) {

        uint8_t ie = emu.memory[0xFFFF];
        uint8_t if_reg = emu.memory[0xFF0F];

        ImGui::Text("Interrupt State:");
        ImGui::Text("IME (Master): %s", emu.cpu.ime ? "ON" : "OFF");
        ImGui::Text("IE:  $%02X", ie);
        ImGui::Text("IF:  $%02X", if_reg);

        ImGui::Separator();

        // Individual interrupt status
        const char* interrupt_names[] = { "VBlank", "LCD STAT", "Timer", "Serial", "Joypad" };
        for (int i = 0; i < 5; i++) {
            bool enabled = (ie >> i) & 1;
            bool pending = (if_reg >> i) & 1;

            ImGui::Text("%s: Enable=%s Pending=%s",
                interrupt_names[i],
                enabled ? "Y" : "N",
                pending ? "Y" : "N");

            if (enabled && pending) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1, 0, 0, 1), "[FIRING]");
            }
        }

        ImGui::Separator();

        // Interrupt history
        if (!timing_paused) {
            interrupt_history.erase(interrupt_history.begin());
            interrupt_history.push_back(if_reg);
        }

        ImGui::Text("Interrupt History:");
        ImGui::PlotLines("##Interrupts",
            reinterpret_cast<float*>(interrupt_history.data()),
            interrupt_history.size(), 0, nullptr, 0, 31, ImVec2(400, 80));

        // Manual interrupt control
        ImGui::Separator();
        ImGui::Text("Manual Control:");

        if (ImGui::Button("Trigger VBlank")) {
            emu.memory[0xFF0F] |= 0x01;
        }
        ImGui::SameLine();
        if (ImGui::Button("Trigger STAT")) {
            emu.memory[0xFF0F] |= 0x02;
        }
        ImGui::SameLine();
        if (ImGui::Button("Trigger Timer")) {
            emu.memory[0xFF0F] |= 0x04;
        }

        if (ImGui::Button("Clear All IF")) {
            emu.memory[0xFF0F] = 0;
        }
    }
    ImGui::End();
}

// 4. Cycle-Accurate Stepper
void DrawCycleStepperWindow(Emulator& emu) {
    if (ImGui::Begin("Cycle Stepper")) {

        ImGui::Text("CPU Cycles: %llu", emu.cpu.cycles);
        ImGui::Text("PC: $%04X", emu.cpu.PC);

        // Step controls
        ImGui::SliderInt("Step Count", &step_count, 1, 1000);

        if (ImGui::Button("Step Instruction")) {
            for (int i = 0; i < step_count; i++) {
                emu.step();
            }
        }

        ImGui::SameLine();
        if (ImGui::Button("Step Scanline")) {
            int target_ly = (emu.ppu.ly + 1) % 154;
            while (emu.ppu.ly != target_ly && emu.running) {
                emu.step();
            }
        }

        ImGui::SameLine();
        if (ImGui::Button("Step Frame")) {
            emu.runFrame();
        }

        ImGui::Separator();

        // Breakpoints
        ImGui::Text("Breakpoints:");
        ImGui::InputInt("Break at LY", &breakpoint_scanline);
        ImGui::InputInt("Break at Cycle", &breakpoint_cycle);

        if (ImGui::Button("Set LY Breakpoint")) {
            // You'll need to implement this in your step() function
            // Check if emu.ppu.ly == breakpoint_scanline and pause
        }

        ImGui::Separator();

        // Current instruction preview
        if (emu.rom.size() > emu.cpu.PC) {
            uint8_t opcode = emu.readMemory(emu.cpu.PC);
            ImGui::Text("Next Instruction: $%02X", opcode);

            // Add instruction names for common opcodes
            static const char* instruction_names[256] = {
                "NOP", "LD BC,nn", "LD (BC),A", "INC BC", "INC B", "DEC B", "LD B,n", "RLCA",
                // ... add more as needed
            };

            if (instruction_names[opcode]) {
                ImGui::Text("Mnemonic: %s", instruction_names[opcode]);
            }
        }
    }
    ImGui::End();
}

// 5. Memory Access Monitor
void DrawMemoryAccessMonitor(Emulator& emu) {
    if (ImGui::Begin("Memory Access Monitor")) {

        static std::vector<std::pair<uint16_t, uint8_t>> recent_reads;
        static std::vector<std::pair<uint16_t, uint8_t>> recent_writes;
        static int max_history = 50;

        ImGui::SliderInt("History Size", &max_history, 10, 200);

        // You'll need to modify your readMemory/writeMemory to log accesses
        // For now, show current state

        ImGui::Text("Critical I/O Registers:");
        ImGui::Text("LCDC ($FF40): $%02X", emu.memory[0xFF40]);
        ImGui::Text("STAT ($FF41): $%02X", emu.memory[0xFF41]);
        ImGui::Text("LY   ($FF44): $%02X", emu.memory[0xFF44]);
        ImGui::Text("LYC  ($FF45): $%02X", emu.memory[0xFF45]);
        ImGui::Text("DIV  ($FF04): $%02X", emu.memory[0xFF04]);
        ImGui::Text("TIMA ($FF05): $%02X", emu.memory[0xFF05]);
        ImGui::Text("IF   ($FF0F): $%02X", emu.memory[0xFF0F]);
        ImGui::Text("IE   ($FFFF): $%02X", emu.memory[0xFFFF]);

        ImGui::Separator();

        // Register editor
        ImGui::Text("Register Editor:");
        static int edit_addr = 0xFF40;
        static int edit_value = 0;

        ImGui::InputInt("Address", &edit_addr, 1, 16, ImGuiInputTextFlags_CharsHexadecimal);
        edit_addr = std::max(0, std::min(0xFFFF, edit_addr));

        if (edit_addr >= 0 && edit_addr <= 0xFFFF) {
            edit_value = emu.memory[edit_addr];
            if (ImGui::InputInt("Value", &edit_value, 1, 16, ImGuiInputTextFlags_CharsHexadecimal)) {
                edit_value = std::max(0, std::min(0xFF, edit_value));
                emu.writeMemory(edit_addr, edit_value);
            }
        }
    }
    ImGui::End();
}

// 6. Test ROM Helper Window
void DrawTestROMHelper(Emulator& emu) {
    if (ImGui::Begin("Test ROM Helper")) {

        ImGui::Text("Common Test ROM Checks:");

        if (ImGui::Button("Check Timer Frequency")) {
            // Reset timer and check if it's incrementing at correct rate
            emu.memory[0xFF05] = 0; // Reset TIMA
            emu.memory[0xFF07] = 0x04; // Enable timer, 4096 Hz

            ImGui::TextWrapped("Timer reset. Watch TIMA increment rate in Timer Analysis window.");
        }

        if (ImGui::Button("Test LCD Turn-On Timing")) {
            // Turn off LCD, wait, then turn on at specific timing
            emu.ppu.lcdc &= ~0x80; // Turn off LCD
            emu.memory[0xFF40] = emu.ppu.lcdc;

            // Step to specific point in scanline
            while (emu.cycles_in_scanline < 200) {
                emu.step();
            }

            // Turn on LCD
            emu.ppu.lcdc |= 0x80;
            emu.memory[0xFF40] = emu.ppu.lcdc;

            ImGui::TextWrapped("LCD turned on mid-scanline. Check PPU timing window.");
        }

        ImGui::Separator();

        // Test-specific helpers
        ImGui::Text("Blargg Test Helpers:");

        if (ImGui::Button("Skip to Serial Output")) {
            // Many Blargg tests output results via serial port
            // Look for writes to $FF01/$FF02
            ImGui::TextWrapped("Watch for serial output at $FF01 (data) and $FF02 (control)");
        }

        if (ImGui::Button("Monitor Test Status")) {
            // Some tests write status to specific memory locations
            uint8_t status = emu.memory[0xA000]; // Example location
            ImGui::Text("Test Status Byte: $%02X", status);
        }
    }
    ImGui::End();
}

// Main function to call all debug windows
void DrawAllDebugWindows(Emulator& emu) {
    DrawPPUTimingWindow(emu);
    DrawTimerAnalysisWindow(emu);
    DrawInterruptMonitor(emu);
    DrawCycleStepperWindow(emu);
    DrawMemoryAccessMonitor(emu);
    DrawTestROMHelper(emu);
}


int main() {
    // === Window Setup ===
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // Request OpenGL 3.3 Core Profile
    const char* glsl_version = "#version 330";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window
    GLFWwindow* window = glfwCreateWindow(2100, 1030, "Game Boy Emulator", NULL, NULL);
    if (window == NULL) {
        std::cerr << "Failed to create GLFW window!" << std::endl;
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync for smooth rendering

    // === Emulator Setup ===
    Emulator gb;  // Create our Game Boy emulator instance
    g_emulator = &gb; // Set global pointer for key callback

    glfwSetWindowUserPointer(window, &gb);
    glfwSetKeyCallback(window, key_callback);

    // Load OpenGL functions
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize OpenGL loader!\n";
        return 1;
    }

    // === Dear ImGui Setup ===
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_DockingEnable; // Docking disabled for now

    ImGui::StyleColorsDark();  // Use dark theme
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // GUI state variables - control which windows are visible
    bool show_cpu_debug = true;     // Show CPU registers and state
    bool show_memory_viewer = true; // Show memory hex dump
    bool show_ppu_debug = true;     // Show graphics chip state
    bool show_screen = true;        // Show Game Boy screen
    bool show_controls = true;      // Show emulator controls
    bool show_tile_debug = true;        // Show tile data visualization
    bool show_vram_hex = true;           // Show VRAM hex dump
    bool show_graphics_debug = true;    // Show graphics analysis
    bool show_timer_debug = true;    // Show timing analysis

    // Performance tracking variables
    float fps = 0.0f;           // Frames per second counter
    int frame_count = 0;        // Frame counter for FPS calculation
    double last_time = glfwGetTime(); // Last time we calculated FPS

    // === Main Loop ===
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();  // Handle window events (close, resize, etc.)

        // Calculate FPS every second
        double current_time = glfwGetTime();
        frame_count++;
        if (current_time - last_time >= 1.0) {  // One second elapsed
            fps = frame_count / (current_time - last_time);
            frame_count = 0;
            last_time = current_time;
        }

        // === Emulator Execution ===
        // Run emulator for one frame if it's running
        if (gb.isRunning()) {
            gb.runFrame(); // Execute ~70224 CPU cycles (one Game Boy frame)
        }

        // === ImGui Rendering Setup ===
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Create main window layout (without docking)
        // ImGui::DockSpaceOverViewport(ImGui::GetMainViewport()); // Docking disabled

        // === Menu Bar ===
        // Top menu bar with File, Debug, and View menus
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Load ROM...")) {
                    const char* filter_patterns[] = { "*.gb" };
                    const char* filename = tinyfd_openFileDialog(
                        "Select Game Boy ROM", "", 1, filter_patterns, "Game Boy ROMs", 0);

					// If a file was selected, load it into the emulator
                    if (filename) {
                        gb.loadROM(filename);
                        //gb.debug8800MethodTiles();
                    }
                }
                if (ImGui::MenuItem("Load Test Program")) {
                    // Load our built-in test program for learning
                    //gb.loadTestProgram();
                    gb.loadTestPattern();
                }
                if (ImGui::MenuItem("Reset")) {
                    gb.reset();  // Reset emulator to initial state
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Exit")) {
                    glfwSetWindowShouldClose(window, true);
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Debug")) {
                ImGui::MenuItem("CPU State", NULL, &show_cpu_debug);
                ImGui::MenuItem("PPU State", NULL, &show_ppu_debug);
                ImGui::MenuItem("Memory Viewer", NULL, &show_memory_viewer);
                ImGui::MenuItem("Tile Debug", NULL, &show_tile_debug);      
                ImGui::MenuItem("VRAM Hex", NULL, &show_vram_hex);           
                ImGui::MenuItem("Graphics Debug", NULL, &show_graphics_debug);
				ImGui::MenuItem("Timing Analysis", NULL, &show_timer_debug);
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("View")) {
                // Toggle display windows on/off
                ImGui::MenuItem("Screen", NULL, &show_screen);
                ImGui::MenuItem("Controls", NULL, &show_controls);
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        // === Control Panel Window ===
        // Main controls for running/stepping the emulator
        if (show_controls) {
            ImGui::Begin("Controls", &show_controls);

            // Play/Pause button (changes text based on current state)
            if (ImGui::Button(gb.isRunning() ? "Pause" : "Resume")) {
                if (gb.isRunning()) {
                    gb.pause();   // Stop continuous execution
                }
                else {
                    gb.resume();  // Start continuous execution
                }
            }
            ImGui::SameLine();

            // Step button - execute exactly one instruction
            if (ImGui::Button("Step")) {
                gb.step();  // Execute one CPU instruction and update
            }
            ImGui::SameLine();

            // Reset button - restart emulator
            if (ImGui::Button("Reset")) {
                gb.reset();
            }

            ImGui::Separator();

            // Performance information
            ImGui::Text("Performance:");
            ImGui::Text("FPS: %.1f", fps);                    // Frames per second
            ImGui::Text("Cycles: %llu", gb.cpu.cycles);       // Total CPU cycles executed

            ImGui::Separator();

            // Controls help
            ImGui::Text("Game Controls:");
            ImGui::Text("Arrow Keys: D-Pad");
            ImGui::Text("Z: A Button");
            ImGui::Text("X: B Button");
            ImGui::Text("Enter: Start");
            ImGui::Text("Space: Select");

            ImGui::End();
        }

        // === CPU Debug Window ===
        // Shows all CPU registers, flags, and upcoming instructions
        if (show_cpu_debug) {
            ImGui::Begin("CPU State", &show_cpu_debug);

            const auto& cpu = gb.cpu;  // Get reference to CPU state

            ImGui::Text("Registers:");
            // Show 16-bit registers (PC and SP)
            ImGui::Text("PC: $%04X    SP: $%04X", cpu.PC, cpu.SP);

            // Show 8-bit registers in pairs (as they're often used together)
            ImGui::Text("A:  $%02X      F:  $%02X", cpu.A, cpu.F);
            ImGui::Text("B:  $%02X      C:  $%02X", cpu.B, cpu.C);
            ImGui::Text("D:  $%02X      E:  $%02X", cpu.D, cpu.E);
            ImGui::Text("H:  $%02X      L:  $%02X", cpu.H, cpu.L);

            ImGui::Separator();

            // Show individual CPU flags (bits in the F register)
            ImGui::Text("Flags:");
            ImGui::Text("Z: %d  N: %d  H: %d  C: %d",
                (cpu.F >> 7) & 1,  // Zero flag (bit 7)
                (cpu.F >> 6) & 1,  // Add/Sub flag (bit 6)
                (cpu.F >> 5) & 1,  // Half-carry flag (bit 5)
                (cpu.F >> 4) & 1); // Carry flag (bit 4)
            ImGui::Text("IME: %s", cpu.ime ? "enabled" : "disabled"); // Interrupt enable

            ImGui::Separator();

            // Show upcoming instructions for debugging
            ImGui::Text("Next Instructions:");
            for (int i = 0; i < 5; ++i) {
                uint8_t opcode = gb.readMemory(cpu.PC + i);
                ImGui::Text("$%04X: $%02X", cpu.PC + i, opcode);
            }

            ImGui::End();
        }

        // === PPU Debug Window ===
        // Shows graphics chip state and control registers
        if (show_ppu_debug) {
            ImGui::Begin("PPU State", &show_ppu_debug);

            const auto& ppu = gb.ppu;  // Get reference to PPU state

            ImGui::Text("Control Registers:");
            // LCDC controls overall LCD operation
            ImGui::Text("LCDC: $%02X  STAT: $%02X", ppu.lcdc, ppu.stat);

            // Scroll registers control background position
            ImGui::Text("SCY:  $%02X  SCX:  $%02X", ppu.scy, ppu.scx);  // Background scroll

            // Scanline registers
            ImGui::Text("LY:   $%02X  LYC:  $%02X", ppu.ly, ppu.lyc);   // Current line / compare

            // Window registers (overlay layer)
            ImGui::Text("WY:   $%02X  WX:   $%02X", ppu.wy, ppu.wx);    // Window position

            ImGui::Separator();

            // Palette registers control colors
            ImGui::Text("Palettes:");
            ImGui::Text("BGP:  $%02X", ppu.bgp);      // Background palette
            ImGui::Text("OBP0: $%02X", ppu.obp0);     // Sprite palette 0
            ImGui::Text("OBP1: $%02X", ppu.obp1);     // Sprite palette 1

            ImGui::Separator();

            // LCDC breakdown
            ImGui::Text("LCDC Breakdown:");
            ImGui::Text("LCD Enable: %s", (ppu.lcdc & 0x80) ? "ON" : "OFF");
            ImGui::Text("Window Map: %s", (ppu.lcdc & 0x40) ? "9C00-9FFF" : "9800-9BFF");
            ImGui::Text("Window: %s", (ppu.lcdc & 0x20) ? "ON" : "OFF");
            ImGui::Text("BG/Win Tiles: %s", (ppu.lcdc & 0x10) ? "8000-8FFF" : "8800-97FF");
            ImGui::Text("BG Map: %s", (ppu.lcdc & 0x08) ? "9C00-9FFF" : "9800-9BFF");
            ImGui::Text("Sprite Size: %s", (ppu.lcdc & 0x04) ? "8x16" : "8x8");
            ImGui::Text("Sprites: %s", (ppu.lcdc & 0x02) ? "ON" : "OFF");
            ImGui::Text("Background: %s", (ppu.lcdc & 0x01) ? "ON" : "OFF");

            ImGui::End();
        }

        // === VRAM Hex Viewer Window ===
// Specialized hex viewer for video memory
        if (show_vram_hex) {
            ImGui::Begin("VRAM Hex Viewer", &show_vram_hex);

            static int vram_start = 0;
            static int vram_rows = 16;

            // Quick navigation for VRAM regions
            if (ImGui::Button("Tile Data 0##8000")) vram_start = 0x0000;  // 0x8000-0x8FFF
            ImGui::SameLine();
            if (ImGui::Button("Tile Data 1##8800")) vram_start = 0x0800;  // 0x8800-0x97FF
            ImGui::SameLine();
            if (ImGui::Button("Tile Map 0##9800")) vram_start = 0x1800;   // 0x9800-0x9BFF
            ImGui::SameLine();
            if (ImGui::Button("Tile Map 1##9C00")) vram_start = 0x1C00;   // 0x9C00-0x9FFF

            ImGui::SliderInt("VRAM Start", &vram_start, 0, 0x1FE0, "%04X");
            ImGui::SliderInt("Rows", &vram_rows, 8, 32);
            vram_start &= 0xFFF0; // Align to 16

            ImGui::Separator();

            ImGui::BeginChild("VRAMHexDump", ImVec2(0, 0), true);

            // Headers
            ImGui::Text("VRAM   ");
            for (int col = 0; col < 16; ++col) {
                ImGui::SameLine();
                ImGui::Text("%02X ", col);
            }
            ImGui::SameLine();
            ImGui::Text("  ASCII");
            ImGui::Separator();

            // VRAM hex dump
            for (int row = 0; row < vram_rows; ++row) {
                uint16_t vram_addr = vram_start + (row * 16);
                if (vram_addr >= 0x2000) break;

                uint16_t real_addr = 0x8000 + vram_addr;
                ImGui::Text("%04X: ", real_addr);

                char ascii_buf[17] = { 0 };
                for (int col = 0; col < 16; ++col) {
                    ImGui::SameLine();
                    uint16_t addr = vram_addr + col;
                    if (addr < 0x2000) {
                        uint8_t byte = gb.vram[addr];

                        // Color code different regions
                        if (addr < 0x1800) {
                            // Tile data - white
                            ImGui::Text("%02X", byte);
                        }
                        else {
                            // Tile map - yellow
                            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 255, 0, 255));
                            ImGui::Text("%02X", byte);
                            ImGui::PopStyleColor();
                        }

                        ascii_buf[col] = (byte >= 32 && byte < 127) ? byte : '.';
                    }
                    else {
                        ImGui::Text("  ");
                        ascii_buf[col] = ' ';
                    }
                }

                ImGui::SameLine();
                ImGui::Text("  |%s|", ascii_buf);
            }

            ImGui::EndChild();

            ImGui::Separator();
            ImGui::TextColored(ImVec4(1, 1, 1, 1), "White: Tile Data");
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "Yellow: Tile Maps");

            ImGui::End();
        }

        // === Tile Debug Window ===
        // Visual representation of tiles and patterns
        if (show_tile_debug) {
            ImGui::Begin("Tile Debug", &show_tile_debug);

            static int selected_tile = 0;
            static bool show_tile_grid = true;

            ImGui::SliderInt("Tile Index", &selected_tile, 0, 255);
            ImGui::Checkbox("Show Grid", &show_tile_grid);

            ImGui::Separator();

            // Display the selected tile
            ImGui::Text("Tile %d:", selected_tile);

            // Calculate tile address
            uint16_t tile_addr = selected_tile * 16;
            if (tile_addr + 15 < 0x2000) {
                ImGui::BeginChild("TileDisplay", ImVec2(200, 200), true);

                // Draw tile pixel by pixel
                ImDrawList* draw_list = ImGui::GetWindowDrawList();
                ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
                float scale = 20.0f; // 20x20 pixels per Game Boy pixel

                for (int y = 0; y < 8; y++) {
                    uint8_t lo = gb.vram[tile_addr + y * 2];
                    uint8_t hi = gb.vram[tile_addr + y * 2 + 1];

                    for (int x = 0; x < 8; x++) {
                        int bit = 7 - x;
                        uint8_t color = ((hi >> bit) & 1) << 1 | ((lo >> bit) & 1);

                        // Convert Game Boy color to RGB
                        uint32_t rgb_color;
                        switch (color) {
                        case 0: rgb_color = IM_COL32(255, 255, 255, 255); break; // White
                        case 1: rgb_color = IM_COL32(192, 192, 192, 255); break; // Light gray
                        case 2: rgb_color = IM_COL32(96, 96, 96, 255); break;    // Dark gray
                        case 3: rgb_color = IM_COL32(0, 0, 0, 255); break;       // Black
                        }

                        ImVec2 pixel_pos(canvas_pos.x + x * scale, canvas_pos.y + y * scale);
                        draw_list->AddRectFilled(pixel_pos,
                            ImVec2(pixel_pos.x + scale, pixel_pos.y + scale), rgb_color);

                        if (show_tile_grid) {
                            draw_list->AddRect(pixel_pos,
                                ImVec2(pixel_pos.x + scale, pixel_pos.y + scale),
                                IM_COL32(128, 128, 128, 255));
                        }
                    }
                }

                ImGui::EndChild();
            }
            else {
                ImGui::Text("Invalid tile address!");
            }

            ImGui::SameLine();

            // Show tile data in hex
            ImGui::BeginChild("TileHex", ImVec2(200, 200), true);
            ImGui::Text("Hex Data:");

            if (tile_addr + 15 < 0x2000) {
                for (int i = 0; i < 16; i += 2) {
                    uint8_t lo = gb.vram[tile_addr + i];
                    uint8_t hi = gb.vram[tile_addr + i + 1];
                    ImGui::Text("Line %d: %02X %02X", i / 2, lo, hi);
                }
            }

            ImGui::EndChild();

            ImGui::Separator();

            // Quick tile navigation
            ImGui::Text("Quick Select:");
            if (ImGui::Button("Tile 0")) selected_tile = 0;
            ImGui::SameLine();
            if (ImGui::Button("Tile 1")) selected_tile = 1;
            ImGui::SameLine();
            if (ImGui::Button("First Non-Zero")) {
                for (int i = 0; i < 256; i++) {
                    uint16_t addr = i * 16;
                    if (addr + 15 >= 0x2000) break;

                    bool has_data = false;
                    for (int j = 0; j < 16; j++) {
                        if (gb.vram[addr + j] != 0) {
                            has_data = true;
                            break;
                        }
                    }
                    if (has_data) {
                        selected_tile = i;
                        break;
                    }
                }
            }

            ImGui::End();
        }

        // === Graphics Debug Window ===
        // Analysis of rendering issues
        if (show_graphics_debug) {
            ImGui::Begin("Graphics Debug", &show_graphics_debug);

            const auto& ppu = gb.ppu;

            ImGui::Text("=== Graphics Analysis ===");

            // Check for common issues
            bool lcd_enabled = ppu.lcdc & 0x80;
            bool bg_enabled = ppu.lcdc & 0x01;
            bool sprites_enabled = ppu.lcdc & 0x02;
            bool bg_tile_data_select = ppu.lcdc & 0x10;
            bool bg_tile_map_select = ppu.lcdc & 0x08;

            ImGui::Text("Display Status:");
            ImGui::TextColored(lcd_enabled ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1),
                "LCD: %s", lcd_enabled ? "ON" : "OFF");
            ImGui::TextColored(bg_enabled ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1),
                "Background: %s", bg_enabled ? "ON" : "OFF");
            ImGui::TextColored(sprites_enabled ? ImVec4(0, 1, 0, 1) : ImVec4(0.5, 0.5, 0.5, 1),
                "Sprites: %s", sprites_enabled ? "ON" : "OFF");

            ImGui::Separator();

            ImGui::Text("Memory Layout:");
            ImGui::Text("BG Tile Data: %s", bg_tile_data_select ? "8000-8FFF" : "8800-97FF");
            ImGui::Text("BG Tile Map: %s", bg_tile_map_select ? "9C00-9FFF" : "9800-9BFF");

            ImGui::Separator();

            // Analyze VRAM content
            ImGui::Text("VRAM Analysis:");

            // Count non-zero tiles
            int non_zero_tiles = 0;
            for (int i = 0; i < 384; i++) { // 384 tiles in VRAM
                uint16_t tile_addr = i * 16;
                if (tile_addr + 15 >= 0x2000) break;

                for (int j = 0; j < 16; j++) {
                    if (gb.vram[tile_addr + j] != 0) {
                        non_zero_tiles++;
                        break;
                    }
                }
            }
            ImGui::Text("Non-empty tiles: %d/384", non_zero_tiles);

            // Check tile map
            uint16_t map_base = bg_tile_map_select ? 0x1C00 : 0x1800;
            int non_zero_map_entries = 0;
            for (int i = 0; i < 1024; i++) { // 32x32 tile map
                if (map_base + i < 0x2000 && gb.vram[map_base + i] != 0) {
                    non_zero_map_entries++;
                }
            }
            ImGui::Text("Non-zero map entries: %d/1024", non_zero_map_entries);

            ImGui::Separator();

            // Palette analysis
            ImGui::Text("Palette Analysis:");
            ImGui::Text("BGP: $%02X", ppu.bgp);
            for (int i = 0; i < 4; i++) {
                uint8_t shade = (ppu.bgp >> (i * 2)) & 0x03;
                ImGui::Text("Color %d -> Shade %d", i, shade);
            }

            ImGui::Separator();

            // Current rendering state
            ImGui::Text("Current State:");
            ImGui::Text("Scanline: %d", ppu.ly);
            ImGui::Text("Scroll: X=%d, Y=%d", ppu.scx, ppu.scy);
            ImGui::Text("Window: X=%d, Y=%d", ppu.wx, ppu.wy);

            ImGui::Separator();

            // Debug actions
            if (ImGui::Button("Dump First Tile")) {
                gb.debugTileData(); // Your existing function
            }

            if (ImGui::Button("Force Render")) {
                // Force a full screen re-render
                for (int line = 0; line < 144; line++) {
                    //gb.renderScanline(line);
                }
            }

            ImGui::End();
        }

        ImGui::Separator();

        // Enhanced debug actions
        if (ImGui::Button("Debug Rendering State")) {
            gb.debugRenderingState();
        }

        if (ImGui::Button("Validate VRAM")) {
            gb.validateVRAMData();
        }

        static int debug_tile_index = 0;
        ImGui::InputInt("Debug Tile #", &debug_tile_index, 1, 10);
        debug_tile_index = std::max(0, std::min(383, debug_tile_index));

        if (ImGui::Button("Debug This Tile")) {
            gb.debugSpecificTile(debug_tile_index);
        }

        ImGui::Separator();

        // ROM Analysis section
        if (ImGui::CollapsingHeader("ROM Analysis")) {
            if (gb.rom.size() > 0) {
                ImGui::Text("ROM Size: %zu bytes", gb.rom.size());

                if (gb.rom.size() > 0x150) {
                    // Show ROM header info
                    ImGui::Text("Cartridge Type: $%02X", gb.rom[0x147]);
                    ImGui::Text("ROM Size Code: $%02X", gb.rom[0x148]);
                    ImGui::Text("RAM Size Code: $%02X", gb.rom[0x149]);

                    // Show title
                    char title[17] = { 0 };
                    for (int i = 0; i < 16; i++) {
                        char c = gb.rom[0x134 + i];
                        title[i] = (c >= 32 && c < 127) ? c : '.';
                    }
                    ImGui::Text("Title: %s", title);

                    // Show checksum
                    ImGui::Text("Header Checksum: $%02X", gb.rom[0x14D]);
                    ImGui::Text("Global Checksum: $%04X",
                        (gb.rom[0x14E] << 8) | gb.rom[0x14F]);
                }

                // Check if ROM looks valid
                bool looks_valid = (gb.rom.size() >= 0x8000) &&
                    (gb.rom[0x100] == 0x00 || gb.rom[0x100] == 0xC3);
                ImGui::TextColored(looks_valid ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1),
                    "ROM appears: %s", looks_valid ? "VALID" : "INVALID");
            }
            else {
                ImGui::Text("No ROM loaded");
            }
        }

        // Memory corruption check
        if (ImGui::CollapsingHeader("Memory Corruption Check")) {
            static bool auto_check = false;
            ImGui::Checkbox("Auto-check every frame", &auto_check);

            if (auto_check || ImGui::Button("Check Now")) {
                // Quick corruption detection
                bool corruption_detected = false;
                std::string corruption_report = "";

                // Check for all-zero VRAM (common issue)
                bool all_zero = true;
                for (int i = 0; i < 0x1800; i++) { // Only check tile data area
                    if (gb.vram[i] != 0) {
                        all_zero = false;
                        break;
                    }
                }

                if (all_zero) {
                    corruption_detected = true;
                    corruption_report += "VRAM tile data is all zeros\n";
                }

                // Check for stack overflow into video memory
                if (gb.cpu.SP < 0xC000) {
                    corruption_detected = true;
                    corruption_report += "Stack pointer too low: $" +
                        std::to_string(gb.cpu.SP) + "\n";
                }

                // Display results
                if (corruption_detected) {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "CORRUPTION DETECTED:");
                    ImGui::TextWrapped("%s", corruption_report.c_str());
                }
                else {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Memory appears OK");
                }
            }
        }

        // === Memory Viewer Window ===
        // Hex dump view of Game Boy memory with navigation
        if (show_memory_viewer) {
            ImGui::Begin("Memory Viewer", &show_memory_viewer);

            static int mem_addr = 0;      // Current viewing address
            static int rows_to_show = 16; // How many rows to display

            // Address input with increment buttons
            ImGui::InputInt("Start Address", &mem_addr, 16, 256);
            mem_addr = std::max(0, std::min(0xFFE0, mem_addr)) & 0xFFF0; // Clamp and align to 16

            // Control how many rows to show
            ImGui::SliderInt("Rows", &rows_to_show, 4, 32);

            ImGui::Separator();

            // Quick navigation buttons for important memory regions
            if (ImGui::Button("ROM##0000")) mem_addr = 0x0000;   // Game code
            ImGui::SameLine();
            if (ImGui::Button("VRAM##8000")) mem_addr = 0x8000;  // Video memory
            ImGui::SameLine();
            if (ImGui::Button("RAM##C000")) mem_addr = 0xC000;   // Work RAM
            ImGui::SameLine();
            if (ImGui::Button("I/O##FF00")) mem_addr = 0xFF00;   // Hardware registers

            ImGui::Separator();

            // === Hex Dump Display ===
            ImGui::BeginChild("HexDump", ImVec2(0, 0), true);

            // Column headers (00 01 02 03 ... 0F)
            ImGui::Text("       ");  // Space for address column
            for (int col = 0; col < 16; ++col) {
                ImGui::SameLine();
                ImGui::Text("%02X ", col);
            }
            ImGui::SameLine();
            ImGui::Text("  ASCII");  // ASCII representation column

            ImGui::Separator();

            // Memory rows
            for (int row = 0; row < rows_to_show; ++row) {
                uint16_t row_addr = mem_addr + (row * 16);

                // Address column (shows row start address)
                ImGui::Text("%04X: ", row_addr);

                // Hex columns (16 bytes per row)
                char ascii_buf[17] = { 0 };  // Buffer for ASCII representation
                for (int col = 0; col < 16; ++col) {
                    ImGui::SameLine();
                    uint16_t addr = row_addr + col;
                    if (addr <= 0xFFFF) {
                        uint8_t byte = gb.readMemory(addr);

                        // Color-code special addresses for easier debugging
                        if (addr == gb.cpu.PC) {
                            // Highlight current Program Counter in yellow
                            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 255, 0, 255));
                            ImGui::Text("%02X", byte);
                            ImGui::PopStyleColor();
                        }
                        else if (addr >= gb.cpu.SP && addr < gb.cpu.SP + 8) {
                            // Highlight stack area in cyan
                            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 255, 255, 255));
                            ImGui::Text("%02X", byte);
                            ImGui::PopStyleColor();
                        }
                        else {
                            // Normal white text
                            ImGui::Text("%02X", byte);
                        }

                        // Build ASCII representation (printable chars only)
                        ascii_buf[col] = (byte >= 32 && byte < 127) ? byte : '.';
                    }
                    else {
                        ImGui::Text("  ");  // Empty for invalid addresses
                        ascii_buf[col] = ' ';
                    }
                }

                // ASCII column (shows readable characters)
                ImGui::SameLine();
                ImGui::Text("  |%s|", ascii_buf);
            }

            ImGui::EndChild();

            // Legend explaining the color coding
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "Yellow: Program Counter (PC)");
            ImGui::TextColored(ImVec4(0, 1, 1, 1), "Cyan: Stack Area (SP+)");

            ImGui::End();
        }

        if (show_timer_debug) {
            DrawAllDebugWindows(gb);
        }

        // === Game Boy Screen Display Window ===
        // Shows the actual Game Boy LCD output
        if (show_screen) {
            ImGui::Begin("Game Boy Screen", &show_screen);

            // Convert Game Boy screen buffer to OpenGL texture
            GLuint screen_texture = createScreenTexture(gb.screen);

            // Display at 3x scale (160x144 -> 480x432) for better visibility
            ImVec2 screen_size(160 * 3, 144 * 3);
            ImGui::Image((void*)(intptr_t)screen_texture, screen_size);

            // Show some screen info
            ImGui::Text("Resolution: 160x144 pixels");
            ImGui::Text("Scale: 3x");

            ImGui::End();
        }

        // === Final Rendering ===
        // Render all ImGui windows to the screen
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);  // Dark gray background
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // === Cleanup ===
    // Shut down ImGui and GLFW properly
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    g_emulator = nullptr; // Clear global pointer

    return 0;
}
