#pragma once
#include <windows.h>
#include <gdiplus.h>
#include <vector>
using namespace Gdiplus;
#pragma comment (lib, "Gdiplus.lib")

#define MAZE_VISUAL 1

struct graph_seg_t
{
	float x1, y1;
	float x2, y2;
    graph_seg_t(float x1_, float y1_, float x2_, float y2_) : x1(x1_), y1(y1_), x2(x2_), y2(y2_) {}
};

static std::vector<graph_seg_t> visual_segs;
static float visual_scale;

inline Point map_point(float x, float y)
{
    return Point(x * visual_scale + 150, y * visual_scale + 100);
}

static VOID OnPaint(HDC hdc)
{
    RECT rcCli;
    GetClientRect(WindowFromDC(hdc), &rcCli);
    auto w = rcCli.right - rcCli.left;
    auto h = rcCli.bottom - rcCli.top;
    auto hdcBuffer = CreateCompatibleDC(hdc);
    auto hBitmapBuffer = CreateCompatibleBitmap(hdc, w, h);
    SelectObject(hdcBuffer, hBitmapBuffer);
    Graphics graphics(hdcBuffer);
    graphics.SetCompositingQuality(CompositingQualityHighQuality);
    graphics.SetInterpolationMode(InterpolationModeHighQuality);
    graphics.SetSmoothingMode(SmoothingModeHighQuality);
    graphics.Clear(Color(0xff, 0xea, 0xea, 0xf2));
    Pen pen(Color(0xff, 0x4c, 0x72, 0xb0), 3);
    for (const auto& seg : visual_segs)
    {
        Point points[] = { map_point(seg.x1, seg.y1), map_point(seg.x2, seg.y2) };
        graphics.DrawCurve(&pen, points, 2);
    }
    BitBlt(hdc, 0, 0, w, h, hdcBuffer, 0, 0, SRCCOPY);
    DeleteDC(hdcBuffer);
    DeleteObject(hBitmapBuffer);
}

static LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
static ULONG_PTR gdiplusToken;
static HWND hWnd;

static VOID WINAPI init_visuals(float scaling)
{
    MSG                 msg;
    WNDCLASS            wndClass;
    GdiplusStartupInput gdiplusStartupInput;

    visual_scale = 1600 / scaling * 0.9f;

    // Initialize GDI+.
    GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

    wndClass.style = CS_HREDRAW | CS_VREDRAW;
    wndClass.lpfnWndProc = WndProc;
    wndClass.cbClsExtra = 0;
    wndClass.cbWndExtra = 0;
    wndClass.hInstance = GetModuleHandle(NULL);
    wndClass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
    wndClass.hCursor = LoadCursor(NULL, IDC_ARROW);
    wndClass.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    wndClass.lpszMenuName = NULL;
    wndClass.lpszClassName = TEXT("MazeVisual");

    RegisterClass(&wndClass);

    hWnd = CreateWindow(
        TEXT("MazeVisual"),   // window class name
        TEXT("Render"),  // window caption
        WS_OVERLAPPEDWINDOW,      // window style
        200,            // initial x position
        100,            // initial y position
        1500,            // initial x size
        900,            // initial y size
        NULL,                     // parent window handle
        NULL,                     // window menu handle
        GetModuleHandle(NULL),    // program instance handle
        NULL
    );

    ShowWindow(hWnd, SW_SHOWDEFAULT);
    UpdateWindow(hWnd);

    while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
}

static VOID WINAPI update_visuals(const decltype(visual_segs)& vsegs)
{
    visual_segs = vsegs;
    RECT rect;
    GetClientRect(hWnd, &rect);
    InvalidateRect(hWnd, &rect, FALSE);
    UpdateWindow(hWnd);
    MSG msg;
    while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
}

static INT WINAPI shutdown_visuals()
{
    MSG msg;
    while (GetMessage(&msg, NULL, 0, 0))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    GdiplusShutdown(gdiplusToken);
    return msg.wParam;
}

static LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    HDC          hdc;
    PAINTSTRUCT  ps;

    switch (message)
    {
    case WM_PAINT:
        hdc = BeginPaint(hWnd, &ps);
        OnPaint(hdc);
        EndPaint(hWnd, &ps);
        return 0;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
}

#undef min
#undef max
