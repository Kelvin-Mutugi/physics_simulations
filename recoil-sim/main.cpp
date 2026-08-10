// recoil_sim.cpp
// Terminal-based recoil simulation core for an AK-47.
// Physics: a critically/under-damped spring-damper system represents the
// shooter's body absorbing and returning the rifle after each shot impulse.
// This is the same model you'll drop into raylib later for the visual version.

#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// Tunable constants
// ---------------------------------------------------------------------------

// Rifle + shooter parameters
constexpr double RIFLE_MASS_KG   = 3.6;    // loaded AKM w/ mag, roughly

// --- Three-impulse model, derived in the design log ---
// 1) Muzzle exit: bullet momentum + propellant gas momentum (Hatcher-style
//    two-term free recoil estimate), delivered when the bullet leaves the bore.
// 2) BCG bottoms out: the bolt carrier group, driven rearward by the gas
//    piston, slams into the rear of the receiver and transfers its momentum
//    into the system as a second, smaller impulse.
// 3) BCG returns to battery: the recoil spring drives the BCG back forward;
//    when it locks up front, the sudden deceleration pushes the receiver
//    FORWARD, partially cancelling the first two impulses.
//
// Magnitudes (kg*m/s), derived by hand:
constexpr double P_PRIMARY   = 7.5504; // bullet (0.008kg * 715m/s) + gas (0.0016kg * 1.6*715m/s)
constexpr double P_SECONDARY = 1.75;   // BCG mass 0.5kg * rearward velocity 3.5m/s
constexpr double P_TERTIARY  = -2.0;   // BCG mass 0.5kg * return velocity 4.0m/s, FORWARD (negative)

// Timing (seconds), derived from barrel length / gas port position / BCG travel:
constexpr double T_PRIMARY_DELAY   = 0.00116;              // bullet transit to muzzle exit
constexpr double T_SECONDARY_DELAY = 0.00076 + 0.02571;    // gas port uncovered + BCG rearward travel
constexpr double T_TERTIARY_DELAY  = T_SECONDARY_DELAY + 0.015; // + ~15ms forward return stroke

// Spring-damper "shoulder/arm" response — NOT physical rifle constants,
// these represent how stiffly the shooter's body resists and returns motion.
constexpr double STIFFNESS_K = 900.0;   // N/m equivalent, higher = snappier return
constexpr double DAMPING_C   = 55.0;    // higher = less oscillation/wobble

// --- Angular (muzzle rise) axis ---
// Bore-axis-to-shoulder offset: this is what converts the linear recoil
// force into a torque (tau = F * d), since the force doesn't act through
// the shoulder contact point.
constexpr double BORE_TO_SHOULDER_OFFSET_M = 0.05;

// Moment of inertia: rifle approximated as a rod of mass RIFLE_MASS_KG and
// length AK_LENGTH_M, rotating about one end (the shoulder pocket).
// I = (1/3) * m * L^2
constexpr double AK_LENGTH_M = 0.87;
constexpr double MOMENT_OF_INERTIA = (1.0 / 3.0) * RIFLE_MASS_KG * AK_LENGTH_M * AK_LENGTH_M;

// Angular spring-damper constants are NOT guessed independently — they're
// back-derived from the linear system's natural frequency and damping ratio,
// so both axes share the same underlying "personality" even though the
// numbers look completely different (rotational units vs linear units).
//   omega_n = sqrt(K / m)              -> natural frequency of linear system
//   zeta    = C / (2 * sqrt(K * m))    -> damping ratio of linear system
//   K_theta = omega_n^2 * I
//   C_theta = 2 * zeta * omega_n * I
constexpr double LINEAR_OMEGA_N = 15.8114; // sqrt(900/3.6)
constexpr double LINEAR_ZETA    = 0.4830;  // 55 / (2*sqrt(900*3.6))
constexpr double STIFFNESS_K_THETA = LINEAR_OMEGA_N * LINEAR_OMEGA_N * MOMENT_OF_INERTIA;
constexpr double DAMPING_C_THETA   = 2.0 * LINEAR_ZETA * LINEAR_OMEGA_N * MOMENT_OF_INERTIA;

// Cyclic rate
constexpr double RPM = 600.0;
constexpr double SHOT_INTERVAL_S = 60.0 / RPM; // ~0.1s between shots, full auto

// Simulation stepping
constexpr double DT = 0.001;            // 1ms physics step
constexpr double SIM_DURATION_S = 1.2;  // how long to run per demo

// ---------------------------------------------------------------------------
// Recoil model
// ---------------------------------------------------------------------------

// A single future impulse: "at this absolute simulation time, apply this
// much momentum to the receiver." Using absolute timestamps (not countdowns)
// means firing a new shot never has to touch any already-pending event —
// they all just compare themselves against one shared clock.
struct ScheduledImpulse {
    double triggerTime; // absolute sim time (seconds) when this fires
    double impulse;     // kg*m/s, positive = rearward, negative = forward
};

struct RecoilState {
    double x = 0.0;   // displacement (muzzle rise proxy, arbitrary units)
    double v = 0.0;   // velocity of that displacement
    double simTime = 0.0; // running clock

    std::vector<ScheduledImpulse> pending;
    bool verbose = true; // print ">>> impulse ..." lines as events land

    // Advance the spring-damper system by dt seconds, applying any impulses
    // that have come due.
    void update(double dt) {
        simTime += dt;

        // Apply any impulses whose time has arrived, then drop them.
        // (Simple linear scan — fine for the handful of pending events
        // we'll ever have at once. No need for anything fancier here.)
        for (auto it = pending.begin(); it != pending.end(); ) {
            if (simTime >= it->triggerTime) {
                if (verbose) {
                    std::cout << ">>> impulse " << std::fixed << std::setprecision(4)
                              << it->impulse << " kg*m/s at t=" << simTime * 1000.0 << "ms\n";
                }
                v += it->impulse / RIFLE_MASS_KG;
                it = pending.erase(it);
            } else {
                ++it;
            }
        }

        // Free response of the spring-damper between impulses.
        double accel = (-STIFFNESS_K * x - DAMPING_C * v) / RIFLE_MASS_KG;
        v += accel * dt;
        x += v * dt;
    }

    // Trigger pull: schedule the three-impulse sequence for this shot,
    // timed relative to the current simulation clock.
    void fire() {
        pending.push_back({simTime + T_PRIMARY_DELAY,   P_PRIMARY});
        pending.push_back({simTime + T_SECONDARY_DELAY, P_SECONDARY});
        pending.push_back({simTime + T_TERTIARY_DELAY,  P_TERTIARY});
    }
};

// --- Angular counterpart to RecoilState ---
// Same three-event timing, same scheduling logic, but tracks rotation (theta,
// omega) instead of displacement (x, v), driven by torque impulses instead
// of force impulses. Kept as a separate struct (rather than a shared
// template) so the two axes stay easy to read and compare independently.
struct AngularRecoilState {
    double theta = 0.0;  // angle (radians), muzzle rise proxy
    double omega = 0.0;  // angular velocity (rad/s)
    double simTime = 0.0;

    std::vector<ScheduledImpulse> pending; // .impulse here means angular impulse (L)
    bool verbose = true;

    void update(double dt) {
        simTime += dt;

        for (auto it = pending.begin(); it != pending.end(); ) {
            if (simTime >= it->triggerTime) {
                if (verbose) {
                    std::cout << ">>> angular impulse " << std::fixed << std::setprecision(4)
                              << it->impulse << " kg*m^2/s at t=" << simTime * 1000.0 << "ms\n";
                }
                omega += it->impulse / MOMENT_OF_INERTIA;
                it = pending.erase(it);
            } else {
                ++it;
            }
        }

        // I*theta'' + C_theta*theta' + K_theta*theta = 0 (free response)
        double angAccel = (-STIFFNESS_K_THETA * theta - DAMPING_C_THETA * omega) / MOMENT_OF_INERTIA;
        omega += angAccel * dt;
        theta += omega * dt;
    }

    // Torque impulse (L) = force impulse (p) * bore-to-shoulder offset (d).
    // Same three timestamps as the linear system, just scaled magnitudes.
    void fire() {
        pending.push_back({simTime + T_PRIMARY_DELAY,   P_PRIMARY   * BORE_TO_SHOULDER_OFFSET_M});
        pending.push_back({simTime + T_SECONDARY_DELAY, P_SECONDARY * BORE_TO_SHOULDER_OFFSET_M});
        pending.push_back({simTime + T_TERTIARY_DELAY,  P_TERTIARY  * BORE_TO_SHOULDER_OFFSET_M});
    }
};

// ---------------------------------------------------------------------------
// Terminal visualization
// ---------------------------------------------------------------------------

// Draws a horizontal bar centered at 0, with the current displacement marked.
// Generic enough to reuse for both linear (x) and angular (theta) axes —
// just pass a different maxRange to fit each axis's typical scale.
void drawBar(const char* label, double value, double rate, double maxRange) {
    constexpr int WIDTH = 41; // odd, so there's a true center column
    int center = WIDTH / 2;

    double normalized = value / maxRange;      // roughly -1..1
    int pos = center + static_cast<int>(std::round(normalized * center));
    pos = std::max(0, std::min(WIDTH - 1, pos));

    std::string bar(WIDTH, '-');
    bar[center] = '|';   // rest position marker
    bar[pos] = '#';       // current position

    std::cout << label << " [" << bar << "] " << std::fixed << std::setprecision(4) << value
              << "  rate=" << std::setprecision(3) << rate << "\n";
}

// ---------------------------------------------------------------------------
// Demo modes
// ---------------------------------------------------------------------------

void runSingleShot() {
    std::cout << "=== Single shot (linear + angular) ===\n";
    RecoilState rifle;
    AngularRecoilState muzzle;
    rifle.fire();
    muzzle.fire();

    int steps = static_cast<int>(SIM_DURATION_S / DT);
    int printEvery = 5; // 5ms — fine enough to see all three impulses land

    for (int i = 0; i < steps; ++i) {
        rifle.update(DT);
        muzzle.update(DT);
        if (i % printEvery == 0) {
            drawBar("lin", rifle.x, rifle.v, 0.05);
            drawBar("ang", muzzle.theta, muzzle.omega, 0.05);
        }
    }
}

void runFullAuto(double burstDurationS) {
    std::cout << "\n=== Full auto, " << burstDurationS << "s burst (linear + angular) ===\n";
    RecoilState rifle;
    AngularRecoilState muzzle;

    double nextShotTime = 0.0;
    int steps = static_cast<int>((burstDurationS + 0.5) / DT); // tail for settle
    int printEvery = 10;

    for (int i = 0; i < steps; ++i) {
        if (rifle.simTime >= nextShotTime && rifle.simTime <= burstDurationS) {
            rifle.fire();
            muzzle.fire();
            nextShotTime += SHOT_INTERVAL_S;
        }
        rifle.update(DT);
        muzzle.update(DT);
        if (i % printEvery == 0) {
            drawBar("lin", rifle.x, rifle.v, 0.05);
            drawBar("ang", muzzle.theta, muzzle.omega, 0.05);
        }
    }
}

int main() {
    std::cout << "AK-47 recoil model (terminal preview)\n";
    std::cout << "Displacement is an arbitrary muzzle-rise proxy, not calibrated to degrees yet.\n\n";

    runSingleShot();

    // 3-round burst at 600rpm = ~0.2s of trigger time
    runFullAuto(0.2);

    return 0;
}