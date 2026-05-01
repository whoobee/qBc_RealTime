#pragma once
// =============================================================================
// tof_filter.h — Per-slot TOF distance filter (header-only)
// =============================================================================
// Pipeline applied to each new sample:
//   median(3) → EMA(alpha) → short hold → publish "no target"
//
//   median(3)        kills isolated spikes / multipath outliers
//   EMA(alpha)       smooths Gaussian jitter on valid readings
//   hold window      keeps last valid value across brief signal dropouts
//                    (target still there, sensor flickered) — short, so
//                    we don't republish stale noise readings as real
//                    measurements
//
// After the hold expires we step directly to TOF_MAX_MM and `valid()` goes
// false. We do NOT linearly interpolate from _lastValid → MAX: those
// intermediate distances are physically meaningless and downstream
// consumers (frontend / BT / nav) would treat them as real targets.
//
// Tuning (at the 20 Hz perception tick):
//   HOLD_SAMPLES  = 6   →  0.3 s hold of last valid value
//   EMA_ALPHA     = 0.3 →  ~3-sample time constant (~150 ms smoothing)
//
// Output sentinel: TOF_MAX_MM (65535) means "no confident target / out of
// range" — well above OBSTACLE_WARNING_MM, so safety treats it as clear.
// `valid()` returns false in that state so callers (e.g. task_comm) can
// publish NaN and have the bridge map it to JSON null.
// =============================================================================

#include <Arduino.h>

class TOFFilter {
public:
    static constexpr uint16_t TOF_MAX_MM      = 65535;
    static constexpr uint8_t  MEDIAN_WINDOW   = 3;
    static constexpr uint16_t HOLD_SAMPLES    = 6;     // 0.3 s @ 20 Hz
    static constexpr float    EMA_ALPHA       = 0.3f;

    TOFFilter() { reset(); }

    void reset() {
        _medFilled = 0;
        _medIdx    = 0;
        _emaInit   = false;
        _emaValue  = 0.0f;
        _lastValid = TOF_MAX_MM;
        _missCount = HOLD_SAMPLES + 1; // cold start = no confidence
        _valid     = false;
    }

    // Push a new sample. Returns the published mm value. After the hold
    // window expires the value steps directly to TOF_MAX_MM and `valid()`
    // returns false — callers must treat that as "no measurement" rather
    // than a real distance.
    uint16_t update(bool valid, uint16_t mm) {
        if (valid) {
            _medBuf[_medIdx] = mm;
            _medIdx = (uint8_t)((_medIdx + 1) % MEDIAN_WINDOW);
            if (_medFilled < MEDIAN_WINDOW) _medFilled++;

            uint16_t med = medianOfWindow();

            if (!_emaInit) {
                _emaValue = (float)med;
                _emaInit  = true;
            } else {
                _emaValue = EMA_ALPHA * (float)med + (1.0f - EMA_ALPHA) * _emaValue;
            }

            _lastValid = (uint16_t)_emaValue;
            _missCount = 0;
            _valid     = true;
            return _lastValid;
        }

        if (_missCount <= HOLD_SAMPLES) _missCount++;

        if (_missCount <= HOLD_SAMPLES) {
            // Brief dropout — keep republishing _lastValid.
            _valid = true;
            return _lastValid;
        }
        // Hold expired — emit "no target" sentinel and mark invalid so
        // task_comm publishes NaN to the bridge.
        _valid = false;
        return TOF_MAX_MM;
    }

    // True when the most recent update() produced a real (or briefly
    // held) measurement. False once the hold has expired.
    bool valid() const { return _valid; }

    // Coarse confidence indicator for debug / telemetry.
    float confidence() const {
        if (_missCount == 0) return 1.0f;
        if (_missCount > HOLD_SAMPLES) return 0.0f;
        return 1.0f - (float)_missCount / (float)HOLD_SAMPLES;
    }

private:
    uint16_t medianOfWindow() const {
        // Sort copy of the filled portion; pick middle element.
        uint16_t tmp[MEDIAN_WINDOW];
        for (uint8_t i = 0; i < _medFilled; i++) tmp[i] = _medBuf[i];
        for (uint8_t i = 1; i < _medFilled; i++) {
            uint16_t key = tmp[i];
            int8_t j = (int8_t)i - 1;
            while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
            tmp[j + 1] = key;
        }
        return tmp[_medFilled / 2];
    }

    uint16_t _medBuf[MEDIAN_WINDOW];
    uint8_t  _medFilled;
    uint8_t  _medIdx;
    bool     _emaInit;
    float    _emaValue;
    uint16_t _lastValid;
    uint16_t _missCount;
    bool     _valid;
};
