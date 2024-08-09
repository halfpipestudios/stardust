#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#define WIN32_EXTRA_LEAN

#include <windows.h>
#include <xinput.h>
#include <stdio.h>
#include <stdlib.h>

#include <sd_platform.h>
#include <sd_input.h>

#pragma comment(lib, "User32.lib")
#pragma comment(lib, "Gdi32.lib")
#pragma comment(lib, "Xinput.lib")
LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);

static WORD XInputButtons[] = {
    XINPUT_GAMEPAD_DPAD_UP,
    XINPUT_GAMEPAD_DPAD_DOWN,
    XINPUT_GAMEPAD_DPAD_LEFT,
    XINPUT_GAMEPAD_DPAD_RIGHT,
    XINPUT_GAMEPAD_START,
    XINPUT_GAMEPAD_BACK,
    XINPUT_GAMEPAD_A,
    XINPUT_GAMEPAD_B,
    XINPUT_GAMEPAD_X,
    XINPUT_GAMEPAD_Y
};

static HWND window;
static u32 window_width;
static u32 window_height;
static HDC hdc;
static HBITMAP back_buffer_handle;
static u32 *back_buffer;
static f32 *depth_buffer;
static bool should_quit;
static i32 vsynch;
static SDMemory memory;

static LARGE_INTEGER frequency;

static void sd_window_open(char *title, u32 width, u32 height) {
    HINSTANCE instance = GetModuleHandle(0);

    WNDCLASSEX wndclass;
    wndclass.cbSize = sizeof(WNDCLASSEX);
    wndclass.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    wndclass.lpfnWndProc = WndProc;
    wndclass.cbClsExtra = 0;
    wndclass.cbWndExtra = 0;
    wndclass.hInstance = instance;
    wndclass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
    wndclass.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
    wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
    wndclass.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
    wndclass.lpszMenuName = 0;
    wndclass.lpszClassName = title;
    RegisterClassEx(&wndclass);

    int screen_width  = GetSystemMetrics(SM_CXSCREEN);
    int screen_height = GetSystemMetrics(SM_CYSCREEN);
    int client_width  = width;
    int client_height = height;
    RECT window_rect;
    SetRect(&window_rect, 
            (screen_width/2) - (client_width/2),
            (screen_height/2) - (client_height/2),
            (screen_width/2) + (client_width/2),
            (screen_height/2) + (client_height/2));

    DWORD style = (WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_MAXIMIZEBOX);
    AdjustWindowRectEx(&window_rect, style, FALSE, 0);
    HWND hwnd = CreateWindowEx(0, wndclass.lpszClassName,
        title, style,
        window_rect.left,
        window_rect.top,
        window_rect.right - window_rect.left,
        window_rect.bottom - window_rect.top,
        nullptr, nullptr, instance, nullptr);

    window = hwnd;
    window_width = width;
    window_height = height;
    hdc = GetDC(hwnd);

    SD_INFO("Window Created!");
    ShowWindow(hwnd, SW_SHOW);
}

void sd_init(char *title, u32 width, u32 height) {

    sd_window_open(title, width, height);

    BITMAPINFO bitmap_info = {};
    bitmap_info.bmiHeader.biSize = sizeof(bitmap_info.bmiHeader);
    bitmap_info.bmiHeader.biWidth = window_width;
    bitmap_info.bmiHeader.biHeight = window_height;
    bitmap_info.bmiHeader.biPlanes = 1;
    bitmap_info.bmiHeader.biBitCount = 32;
    bitmap_info.bmiHeader.biCompression = BI_RGB;
    back_buffer_handle = CreateDIBSection(hdc, &bitmap_info, DIB_RGB_COLORS, (void **)&back_buffer, nullptr, 0);
    if(back_buffer == nullptr) {
        SD_FATAL("Error: Software renderer failed initialization.");
    }
    memset(back_buffer, 0, sizeof(uint32_t) * window_width * window_height);
    depth_buffer = (f32 *)malloc(sizeof(f32) * window_width * window_height);


    // allocate memory for the entire project
    // TODO: pass this to the user side
    memory.size = SD_GB(1);
    memory.used = 0;
    memory.data = (u8 *)malloc(memory.size);
    if(memory.data) {
        SD_INFO("memory allocated: %d bytes", memory.size);
    }

    QueryPerformanceFrequency(&frequency);
}

void sd_shutdown() {
    DeleteObject(back_buffer_handle);
    // TODO: check we this is wrong here ...
    //if(depth_buffer) free(depth_buffer);
    if(memory.data) free(memory.data);
    back_buffer = nullptr;
    depth_buffer = nullptr;
}

void sd_present() {
    HDC back_buffer_dc = CreateCompatibleDC(hdc);
    SelectObject(back_buffer_dc, back_buffer_handle);
    BitBlt(hdc, 0, 0, window_width, window_height, back_buffer_dc, 0, 0, SRCCOPY);
    DeleteDC(back_buffer_dc);
}

static f32 sd_process_xinput_stick(SHORT value, i32 deadZoneValue) {
    f32 result = 0;
    if(value < -deadZoneValue) {
        result = (f32)(value + deadZoneValue) / (32768.0f - deadZoneValue);
    }
    else if(value > deadZoneValue) {
        result = (f32)(value - deadZoneValue) / (32767.0f - deadZoneValue);
    }
    return result;
}


void sd_process_events() {
    MSG msg;
    while(PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    XINPUT_STATE state = {};
    if(XInputGetState(0, &state) == ERROR_SUCCESS)
    {
        XINPUT_GAMEPAD *pad = &state.Gamepad;
        f32 lx =  sd_process_xinput_stick(pad->sThumbLX, XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE);
        f32 ly =  sd_process_xinput_stick(pad->sThumbLY, XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE);
        f32 rx = sd_process_xinput_stick(pad->sThumbRX, XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE);
        f32 ry = sd_process_xinput_stick(pad->sThumbRY, XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE);
        sd_set_left_stick(lx, ly);
        sd_set_right_stick(rx, ry);
    }
}

bool sd_should_close() {
    return should_quit;
}

u32 sd_window_width() {
    return window_width;
}

u32 sd_window_height() {
    return window_height;
}

u32 *sd_back_buffer() {
    return back_buffer;
}

f32 *sd_depth_buffer() {
    return depth_buffer;
}

SDMemory *sd_memory() {
    return &memory;
}

f64 sd_get_time() {
    LARGE_INTEGER current_counter;
    QueryPerformanceCounter(&current_counter);
    f64 time = (f64)current_counter.QuadPart / frequency.QuadPart;
    return time;
}

void sd_sleep(f32 milisecons_to_sleep) {
    Sleep((DWORD)milisecons_to_sleep);
}


void sd_log_message(SDLogType type, const char *message, ...) {
    char buffer[32000];
    uint32_t log_header_length = 14;
    const char *log_header[] = {
        "[SD_FATAL]:   ",
        "[SD_ERROR]:   ",
        "[SD_WARNING]: ",
        "[SD_INFO]:    "
    };
    memcpy(buffer, log_header[type], log_header_length);
    va_list valist;
    va_start(valist, message);
    vsnprintf(buffer + log_header_length, SD_ARRAY_LENGTH(buffer), message, valist);
    va_end(valist);
    HANDLE console_handle = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_SCREEN_BUFFER_INFO console_original_settings;
    GetConsoleScreenBufferInfo(console_handle, &console_original_settings);
    uint8_t colors[4] = { 64, 4, 6, 2 };
    SetConsoleTextAttribute(console_handle, colors[type]);
    printf("%s\n", buffer);
    SetConsoleTextAttribute(console_handle, console_original_settings.wAttributes);
}


static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param)
{    
    switch (msg) 
    {
        case WM_CLOSE: 
        {
            should_quit = true;
        } break;


        case WM_KEYDOWN:
        case WM_KEYUP:
        {
            DWORD key_code = (DWORD)w_param;  
            bool is_down = ((l_param & (1 << 31)) == 0);
            sd_set_key(key_code, is_down);
        } break;

    }
    return DefWindowProc(hwnd, msg, w_param, l_param);
}
