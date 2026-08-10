// recoil_visual.cpp
// raylib front-end for the AK-47 recoil physics core.
//
// The physics (RecoilState, AngularRecoilState, all constants and the
// three-impulse model) are ported UNCHANGED from recoil_sim.cpp so the
// numbers you already validated in the terminal are exactly what drives
// the sprite here. Only the drawBar()/terminal I/O has been replaced with
// a textured, rotating rifle sprite + a live trace graph.
//
// Build (Fedora):
//   sudo dnf install raylib-devel        # if not already installed
//   g++ recoil_visual.cpp -o recoil_visual -lraylib -lm -lpthread -ldl -lrt -lX11
//   ./recoil_visual
//
// Controls:
//   SPACE       fire a single shot
//   F (hold)    full auto (600rpm, matches SHOT_INTERVAL_S below)
//   R           reset both axes to rest
//   [ / ]       decrease / increase linear display scale (px per sim-unit)
//   - / =       decrease / increase angular display scale (deg per radian)
//   ESC         quit

#include "raylib.h"
#include <cmath>
#include <vector>
#include <deque>
#include <string>
#include <cstdio>

// ---------------------------------------------------------------------------
// Physics core — copied verbatim from recoil_sim.cpp. Do not tweak these to
// change how the sprite "feels" — tweak the DISPLAY-ONLY scale constants
// further down instead, so the underlying model stays the one you validated
// in the terminal.
// ---------------------------------------------------------------------------

constexpr double RIFLE_MASS_KG   = 3.6;

constexpr double P_PRIMARY   = 7.5504;
constexpr double P_SECONDARY = 1.75;
constexpr double P_TERTIARY  = -2.0;

constexpr double T_PRIMARY_DELAY   = 0.00116;
constexpr double T_SECONDARY_DELAY = 0.00076 + 0.02571;
constexpr double T_TERTIARY_DELAY  = T_SECONDARY_DELAY + 0.015;

constexpr double STIFFNESS_K = 900.0;
constexpr double DAMPING_C   = 55.0;

constexpr double BORE_TO_SHOULDER_OFFSET_M = 0.05;

constexpr double AK_LENGTH_M = 0.87;
constexpr double MOMENT_OF_INERTIA = (1.0 / 3.0) * RIFLE_MASS_KG * AK_LENGTH_M * AK_LENGTH_M;

constexpr double LINEAR_OMEGA_N = 15.8114;
constexpr double LINEAR_ZETA    = 0.4830;
constexpr double STIFFNESS_K_THETA = LINEAR_OMEGA_N * LINEAR_OMEGA_N * MOMENT_OF_INERTIA;
constexpr double DAMPING_C_THETA   = 2.0 * LINEAR_ZETA * LINEAR_OMEGA_N * MOMENT_OF_INERTIA;

constexpr double RPM = 600.0;
constexpr double SHOT_INTERVAL_S = 60.0 / RPM;

constexpr double DT = 0.001; // fixed 1ms physics step, accumulated from frame time

struct ScheduledImpulse {
    double triggerTime;
    double impulse;
};

struct RecoilState {
    double x = 0.0;
    double v = 0.0;
    double simTime = 0.0;
    std::vector<ScheduledImpulse> pending;

    void update(double dt) {
        simTime += dt;
        for (auto it = pending.begin(); it != pending.end(); ) {
            if (simTime >= it->triggerTime) {
                v += it->impulse / RIFLE_MASS_KG;
                it = pending.erase(it);
            } else {
                ++it;
            }
        }
        double accel = (-STIFFNESS_K * x - DAMPING_C * v) / RIFLE_MASS_KG;
        v += accel * dt;
        x += v * dt;
    }

    void fire() {
        pending.push_back({simTime + T_PRIMARY_DELAY,   P_PRIMARY});
        pending.push_back({simTime + T_SECONDARY_DELAY, P_SECONDARY});
        pending.push_back({simTime + T_TERTIARY_DELAY,  P_TERTIARY});
    }

    void reset() { x = 0.0; v = 0.0; pending.clear(); }
};

struct AngularRecoilState {
    double theta = 0.0;
    double omega = 0.0;
    double simTime = 0.0;
    std::vector<ScheduledImpulse> pending;

    void update(double dt) {
        simTime += dt;
        for (auto it = pending.begin(); it != pending.end(); ) {
            if (simTime >= it->triggerTime) {
                omega += it->impulse / MOMENT_OF_INERTIA;
                it = pending.erase(it);
            } else {
                ++it;
            }
        }
        double angAccel = (-STIFFNESS_K_THETA * theta - DAMPING_C_THETA * omega) / MOMENT_OF_INERTIA;
        omega += angAccel * dt;
        theta += omega * dt;
    }

    void fire() {
        pending.push_back({simTime + T_PRIMARY_DELAY,   P_PRIMARY   * BORE_TO_SHOULDER_OFFSET_M});
        pending.push_back({simTime + T_SECONDARY_DELAY, P_SECONDARY * BORE_TO_SHOULDER_OFFSET_M});
        pending.push_back({simTime + T_TERTIARY_DELAY,  P_TERTIARY  * BORE_TO_SHOULDER_OFFSET_M});
    }

    void reset() { theta = 0.0; omega = 0.0; pending.clear(); }
};

// ---------------------------------------------------------------------------
// DISPLAY-ONLY constants. Nothing above this line changes when you tune
// these — you're only changing how many pixels/degrees represent one
// sim-unit of x / theta, exactly like the terminal's drawBar(maxRange) did.
// ---------------------------------------------------------------------------

// Source image is 2249x654 (aspect ratio ~3.44:1). Rendered width on screen:
constexpr int   IMG_NATIVE_W = 2249;
constexpr int   IMG_NATIVE_H = 654;
constexpr float RENDER_WIDTH_PX = 820.0f; // on-screen sprite width; height follows aspect ratio

// Where the rotation/recoil pivot sits within the sprite, as a fraction of
// its width/height. 0.10 = 10% in from the left edge, roughly the buttstock.
// Move this if your PNG's stock isn't near the left edge.
constexpr float PIVOT_X_FRACTION = 0.10f;
constexpr float PIVOT_Y_FRACTION = 0.50f;

// If your PNG has the muzzle pointing LEFT instead of right, set this false —
// it flips both the horizontal recoil direction and the rotation sign so
// muzzle rise still reads as "barrel tip moves up."
constexpr bool MUZZLE_FACES_RIGHT = true;

// Live-tunable (via keys) starting values. x maxes out around 0.03-0.05 in
// the terminal demo, theta is smaller still — these turn that into visible
// screen motion. Adjust with [ ] and - = while it's running.
float pixelsPerUnitX   = 4000.0f;  // px of translation per 1.0 of sim x
float degreesPerRadian = 8.0f;     // extra multiplier on top of the natural rad->deg conversion

// ---------------------------------------------------------------------------
// Recoil trace graph — small scrolling plot of x(t) and theta(t), same idea
// as the terminal's bar readout but continuous instead of sampled every 5ms.
// ---------------------------------------------------------------------------

struct TracePoint { float t; float x; float theta; };

int main() {
    const int screenW = 1600;
    const int screenH = 900;

    InitWindow(screenW, screenH, "AK-47 Recoil Simulation - raylib viewer");
    SetTargetFPS(144);

    Texture2D rifleTex = LoadTexture("rifle.png");
    if (rifleTex.id == 0) {
        // LoadTexture already logs a raylib error; give a clearer hint too.
        TraceLog(LOG_WARNING, "rifle.png not found next to the executable - "
                               "place it in the same folder as recoil_visual");
    }

    RecoilState        rifle;
    AngularRecoilState muzzle;

    bool autoFire = false;
    double timeSinceLastShot = 1e9;
    double physicsAccumulator = 0.0;

    std::deque<TracePoint> trace;
    const float traceWindowSeconds = 1.5f;
    float wallClock = 0.0f;

    while (!WindowShouldClose()) {
        float frameDt = GetFrameTime();
        wallClock += frameDt;

        // --- input ---
        if (IsKeyPressed(KEY_SPACE)) {
            rifle.fire();
            muzzle.fire();
        }
        autoFire = IsKeyDown(KEY_F);
        if (autoFire) {
            timeSinceLastShot += frameDt;
            if (timeSinceLastShot >= SHOT_INTERVAL_S) {
                rifle.fire();
                muzzle.fire();
                timeSinceLastShot = 0.0;
            }
        } else {
            timeSinceLastShot = 1e9;
        }
        if (IsKeyPressed(KEY_R)) {
            rifle.reset();
            muzzle.reset();
            trace.clear();
        }
        if (IsKeyPressed(KEY_LEFT_BRACKET))  pixelsPerUnitX *= 0.85f;
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) pixelsPerUnitX *= 1.15f;
        if (IsKeyPressed(KEY_MINUS)) degreesPerRadian *= 0.85f;
        if (IsKeyPressed(KEY_EQUAL)) degreesPerRadian *= 1.15f;

        // --- fixed-step physics, accumulated from variable frame time ---
        physicsAccumulator += frameDt;
        while (physicsAccumulator >= DT) {
            rifle.update(DT);
            muzzle.update(DT);
            physicsAccumulator -= DT;
        }
        trace.push_back({wallClock, (float)rifle.x, (float)muzzle.theta});
        while (!trace.empty() && wallClock - trace.front().t > traceWindowSeconds) {
            trace.pop_front();
        }

        // --- derive screen transform from sim state ---
        float dirSign = MUZZLE_FACES_RIGHT ? 1.0f : -1.0f;

        // Recoil pushes the rifle backward relative to the muzzle direction.
        float offsetPx = -dirSign * (float)(rifle.x * pixelsPerUnitX);

        // theta > 0 means muzzle rising in the physics model; on screen with
        // a right-facing muzzle that's a counter-clockwise rotation, which
        // in raylib's clockwise-positive DrawTexturePro rotation is negative.
        float rotationDeg = -dirSign * (float)(muzzle.theta * (180.0 / PI) * degreesPerRadian);

        float renderW = RENDER_WIDTH_PX;
        float renderH = RENDER_WIDTH_PX * ((float)IMG_NATIVE_H / (float)IMG_NATIVE_W);

        // origin is where in the *source rect* the rotation pivot sits,
        // scaled to the destination size (raylib expects origin in dest px).
        Vector2 origin = { renderW * PIVOT_X_FRACTION, renderH * PIVOT_Y_FRACTION };

        // Place the pivot so the sprite's full bounding box (not just the
        // pivot) is centered on screen: left edge = pivotScreen.x - origin.x,
        // so center = pivotScreen.x - origin.x + renderW/2 == screenW/2.
        Vector2 pivotScreen = { screenW * 0.5f + origin.x - renderW * 0.5f, screenH * 0.5f };
        Vector2 spritePos = { pivotScreen.x + offsetPx, pivotScreen.y };

        // --- draw ---
        BeginDrawing();
        ClearBackground(Color{ 24, 26, 30, 255 });

        if (rifleTex.id != 0) {
            Rectangle src = { 0, 0, (float)rifleTex.width, (float)rifleTex.height };
            Rectangle dst = { spritePos.x, spritePos.y, renderW, renderH };
            DrawTexturePro(rifleTex, src, dst, origin, rotationDeg, WHITE);
        } else {
            DrawText("rifle.png not found - drop it next to the executable",
                     40, screenH / 2 - 10, 20, RED);
        }

        if (trace.size() > 1) {
            int gx = 16, gy = screenH - 190, gw = screenW - 32, gh = 160;
            DrawRectangle(gx, gy, gw, gh, Color{ 32, 34, 40, 220 });
            DrawRectangleLines(gx, gy, gw, gh, Color{ 70, 73, 80, 255 });
            DrawLine(gx, gy + gh / 2, gx + gw, gy + gh / 2, Color{ 55, 58, 64, 255 });

            float maxAbsX = 0.001f, maxAbsTheta = 0.001f;
            for (auto &p : trace) {
                maxAbsX = fmaxf(maxAbsX, fabsf(p.x));
                maxAbsTheta = fmaxf(maxAbsTheta, fabsf(p.theta));
            }

            for (size_t i = 1; i < trace.size(); ++i) {
                float t0 = trace[i - 1].t, t1 = trace[i].t;
                float sx0 = gx + gw * (1.0f - (wallClock - t0) / traceWindowSeconds);
                float sx1 = gx + gw * (1.0f - (wallClock - t1) / traceWindowSeconds);

                float sy0x = gy + gh / 2 - (trace[i - 1].x / maxAbsX) * (gh / 2 - 6);
                float sy1x = gy + gh / 2 - (trace[i].x / maxAbsX) * (gh / 2 - 6);
                DrawLineEx({ sx0, sy0x }, { sx1, sy1x }, 2.0f, Color{ 90, 190, 255, 255 });

                float sy0t = gy + gh / 2 - (trace[i - 1].theta / maxAbsTheta) * (gh / 2 - 6);
                float sy1t = gy + gh / 2 - (trace[i].theta / maxAbsTheta) * (gh / 2 - 6);
                DrawLineEx({ sx0, sy0t }, { sx1, sy1t }, 2.0f, Color{ 255, 150, 90, 255 });
            }
            DrawText("x(t)", gx + gw - 60, gy + 4, 14, Color{ 90, 190, 255, 255 });
            DrawText("theta(t)", gx + gw - 60, gy + 20, 14, Color{ 255, 150, 90, 255 });
        }

        EndDrawing();
    }

    if (rifleTex.id != 0) UnloadTexture(rifleTex);
    CloseWindow();
    return 0;
}