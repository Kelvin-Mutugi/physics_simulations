// ============================================================================
// Gun Recoil Simulation — raylib, no game engine
//
// The core idea: recoil is a spring-damper system being repeatedly kicked
// by an impulse. Two coupled oscillators:
//   1. LINEAR   — the gun gets punched backward (Newton's 3rd law)
//   2. ANGULAR  — the bore axis sits ABOVE the pivot (shoulder/grip), so
//                 that backward push also creates a torque -> muzzle rise
//
// Both are just: F = -k*x - c*v  (spring pulls back to rest, damper kills
// the oscillation), integrated every frame.
// ============================================================================

#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Tunable constants — these are exactly the knobs you'll be turning on
// camera during the "does this feel right" part of the video.
// ---------------------------------------------------------------------------
static const float MASS               = 3.5f;   // kg, gun mass
static const float MOMENT_OF_INERTIA  = 0.6f;    // kg*m^2 (approx, about pivot)

static const float SPRING_K_POS   = 900.0f;      // linear spring stiffness
static const float DAMPING_POS    = 55.0f;       // linear damping

static const float SPRING_K_ANGLE = 260.0f;      // angular spring stiffness
static const float DAMPING_ANGLE  = 18.0f;       // angular damping

static const float RECOIL_IMPULSE     = 4.2f;    // Ns, backward kick per shot
static const float TORQUE_IMPULSE     = 3.0f;    // N*m*s, muzzle-rise kick per shot

static const float FIRE_COOLDOWN  = 0.11f;       // seconds between auto-fire shots
static const float BARREL_LENGTH  = 140.0f;      // px, visual only
static const float BARREL_HEIGHT_ABOVE_PIVOT = 10.0f; // px, WHY muzzle rise happens

static const int   GRAPH_SAMPLES = 240;          // rolling history buffer length

// ---------------------------------------------------------------------------
// Recoil state — everything the spring-damper needs
// ---------------------------------------------------------------------------
struct RecoilState {
    Vector2 posOffset   = { 0, 0 };   // linear kickback displacement (px)
    Vector2 linVel      = { 0, 0 };
    float   angleOffset = 0.0f;       // degrees, 0 = resting
    float   angularVel  = 0.0f;       // deg/s

    void Fire() {
        // Kick backward along -x (barrel points +x, bullet exits +x,
        // reaction pushes gun -x)
        linVel.x -= RECOIL_IMPULSE / MASS * 60.0f; // scaled for px-space feel
        // Muzzle rise: angular impulse from the same event
        angularVel -= TORQUE_IMPULSE / MOMENT_OF_INERTIA * 60.0f;
    }

    void Update(float dt) {
        // Linear spring-damper (semi-implicit Euler)
        Vector2 posAccel = {
            (-SPRING_K_POS * posOffset.x - DAMPING_POS * linVel.x) / MASS,
            (-SPRING_K_POS * posOffset.y - DAMPING_POS * linVel.y) / MASS
        };
        linVel.x += posAccel.x * dt;
        linVel.y += posAccel.y * dt;
        posOffset.x += linVel.x * dt;
        posOffset.y += linVel.y * dt;

        // Angular spring-damper
        float angAccel = (-SPRING_K_ANGLE * angleOffset - DAMPING_ANGLE * angularVel)
                          / MOMENT_OF_INERTIA;
        angularVel  += angAccel * dt;
        angleOffset += angularVel * dt;
    }
};

int main() {
    const int screenW = 1280, screenH = 800;
    InitWindow(screenW, screenH, "Recoil Simulation - raylib");
    SetTargetFPS(60);

    RecoilState recoil;
    Vector2 pivot = { 340.0f, 520.0f }; // fixed anchor: shoulder / grip

    float fireCooldownTimer = 0.0f;
    float muzzleFlashTimer  = 0.0f;
    int   shotCount = 0;

    // Rolling buffer of angleOffset for the on-screen recoil-curve graph.
    // This is the single most useful visual for narrating "is this realistic".
    std::vector<float> angleHistory(GRAPH_SAMPLES, 0.0f);
    int historyIndex = 0;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // ---- Input -----------------------------------------------------
        bool wantsFire = IsKeyPressed(KEY_SPACE) || IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
        bool autoFire  = IsKeyDown(KEY_SPACE) || IsMouseButtonDown(MOUSE_BUTTON_LEFT);

        fireCooldownTimer -= dt;
        if ((wantsFire || autoFire) && fireCooldownTimer <= 0.0f) {
            recoil.Fire();
            fireCooldownTimer = FIRE_COOLDOWN;
            muzzleFlashTimer  = 0.05f;
            shotCount++;
        }

        if (IsKeyPressed(KEY_R)) {
            recoil = RecoilState{};
            shotCount = 0;
            std::fill(angleHistory.begin(), angleHistory.end(), 0.0f);
        }

        // ---- Physics update ---------------------------------------------
        recoil.Update(dt);
        muzzleFlashTimer -= dt;

        angleHistory[historyIndex] = recoil.angleOffset;
        historyIndex = (historyIndex + 1) % GRAPH_SAMPLES;

        // ---- Derived draw values -----------------------------------------
        Vector2 gunPos = Vector2Add(pivot, recoil.posOffset);
        float   gunRotation = recoil.angleOffset; // degrees, rest = 0

        // Rectangle origin placed BELOW the bore axis by BARREL_HEIGHT_ABOVE_PIVOT.
        // This is what makes the rotation visually pivot from the grip while
        // the barrel arcs upward -- same reason real muzzle rise happens.
        Rectangle gunRect = { gunPos.x, gunPos.y, BARREL_LENGTH, 26.0f };
        Vector2   origin  = { 0.0f, 13.0f + BARREL_HEIGHT_ABOVE_PIVOT };

        // Muzzle tip world position (for flash + graph reference point)
        float radians = DEG2RAD * gunRotation;
        Vector2 muzzleLocal = { BARREL_LENGTH, -BARREL_HEIGHT_ABOVE_PIVOT };
        Vector2 muzzleWorld = {
            gunPos.x + muzzleLocal.x * cosf(radians) - muzzleLocal.y * sinf(radians),
            gunPos.y + muzzleLocal.x * sinf(radians) + muzzleLocal.y * cosf(radians)
        };

        // ---- Draw ----------------------------------------------------------
        BeginDrawing();
        ClearBackground(Color{ 18, 18, 22, 255 });

        // Ground line for spatial reference
        DrawLine(0, (int)pivot.y + 60, screenW, (int)pivot.y + 60, Color{ 60, 60, 65, 255 });

        // Pivot marker
        DrawCircleV(pivot, 5, Color{ 90, 200, 255, 255 });
        DrawText("PIVOT (shoulder/grip)", (int)pivot.x - 20, (int)pivot.y + 15, 14, Color{90,200,255,255});

        // Gun body
        DrawRectanglePro(gunRect, origin, gunRotation, Color{ 210, 210, 215, 255 });
        // Barrel tip highlight
        DrawCircleV(muzzleWorld, 4, Color{ 150, 150, 155, 255 });

        // Muzzle flash
        if (muzzleFlashTimer > 0.0f) {
            DrawCircleV(muzzleWorld, 14.0f * (muzzleFlashTimer / 0.05f), Color{ 255, 200, 80, 200 });
            DrawCircleV(muzzleWorld, 6.0f, Color{ 255, 240, 180, 255 });
        }

        // ---- Recoil curve graph (bottom of screen) -------------------------
        int graphX = 40, graphY = screenH - 160, graphW = screenW - 80, graphH = 120;
        DrawRectangle(graphX, graphY, graphW, graphH, Color{ 28, 28, 34, 255 });
        DrawRectangleLines(graphX, graphY, graphW, graphH, Color{ 70, 70, 75, 255 });
        DrawText("angleOffset over time (deg)", graphX, graphY - 20, 14, GRAY);

        float maxAngle = 25.0f; // deg, graph vertical scale
        for (int i = 0; i < GRAPH_SAMPLES - 1; i++) {
            int i0 = (historyIndex + i) % GRAPH_SAMPLES;
            int i1 = (historyIndex + i + 1) % GRAPH_SAMPLES;
            float x0 = graphX + (float)i / GRAPH_SAMPLES * graphW;
            float x1 = graphX + (float)(i + 1) / GRAPH_SAMPLES * graphW;
            float y0 = graphY + graphH / 2.0f - (angleHistory[i0] / maxAngle) * (graphH / 2.0f);
            float y1 = graphY + graphH / 2.0f - (angleHistory[i1] / maxAngle) * (graphH / 2.0f);
            DrawLine((int)x0, (int)y0, (int)x1, (int)y1, Color{ 90, 220, 140, 255 });
        }
        DrawLine(graphX, graphY + graphH / 2, graphX + graphW, graphY + graphH / 2, Color{ 60, 60, 65, 255 });

        // ---- HUD ------------------------------------------------------------
        DrawText("SPACE / LEFT CLICK = fire   (hold = auto-fire)   R = reset", 20, 20, 18, RAYWHITE);
        DrawText(TextFormat("Shots fired: %d", shotCount), 20, 46, 18, RAYWHITE);
        DrawText(TextFormat("angleOffset: %.2f deg   angularVel: %.1f deg/s", recoil.angleOffset, recoil.angularVel), 20, 70, 16, Color{170,170,175,255});
        DrawText(TextFormat("posOffset: (%.1f, %.1f)", recoil.posOffset.x, recoil.posOffset.y), 20, 92, 16, Color{170,170,175,255});

        EndDrawing();
    }

    CloseWindow();
    return 0;
}