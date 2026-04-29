#pragma once
// =============================================================================
// tof_filter.h — Per-slot TOF distance filter (header-only)
// =============================================================================
// Pipeline applied to each new sample:
//   median(3) → EMA(alpha) → hold last valid → linear decay to MAX
//
//   median(3)        kills isolated spikes / multipath outliers
//   EMA(alpha)       smooths Gaussian jitter on valid readings
//   hold window      keeps last valid value across brief signal dropouts
//                    (target still there, sensor flickered)
//   decay window     linearly walks the held value toward TOF_MAX_MM after
//                    the hold expires — the longer we go without a valid
//                    sample, the lower our confidence the target is still
//                    there, until we publish "out of range" outright
//
// Tuning (at the 20 Hz perception tick):
//   HOLD_SAMPLES  = 20  →  1.0 s hold of last valid value
//   DECAY_SAMPLES = 40  →  2.0 s linear decay to MAX
//   EMA_ALPHA     = 0.3 →  ~3-sample time constant (~150 ms smoothing)
//
// Output sentinel: TOF_MAX_MM (65535) means "no confident target / out of
// range" — well above OBSTACLE_WARNING_MM, so safety treats it as clear.
// =============================================================================

#include <Arduino.h>

class TOFFilter {
public:
    static constexpr uint16_t TOF_MAX_MM      = 65535;
    static constexpr uint8_t  MEDIAN_WINDOW   = 3;
    static constexpr uint16_t HOLD_SAMPLES    = 20;
    static constexpr uint16_t DECAY_SAMPLES   = 40;
    static constexpr float    EMA_ALPHA       = 0.3f;

    TOFFilter() { reset(); }

    void reset() {
        _medFilled = 0;
        _medIdx    = 0;
        _emaInit   = false;
        _emaValue  = 0.0f;
        _lastValid = TOF_MAX_MM;
        _missCount = HOLD_SAMPLES + DECAY_SAMPLES; // cold start = no confidence
    }

    // Push a new sample. `valid=false` triggers hold-then-decay logic.
    // Returns the published mm value — equal to TOF_MAX_MM once confidence
    // has fully decayed.
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
            return _lastValid;
        }

        if (_missCount < HOLD_SAMPLES + DECAY_SAMPLES) _missCount++;

        if (_missCount <= HOLD_SAMPLES) {
            return _lastValid;
        }
        if (_missCount <= HOLD_SAMPLES + DECAY_SAMPLES) {
            uint16_t into = (uint16_t)(_missCount - HOLD_SAMPLES);
            float t = (float)into / (float)DECAY_SAMPLES;       // 0..1
            float v = (float)_lastValid + t * ((float)TOF_MAX_MM - (float)_lastValid);
            return (uint16_t)v;
        }
        return TOF_MAX_MM;
    }

    // 1.0 = fresh valid sample, 0.0 = fully decayed. Linear in between.
    // Useful for debug / telemetry; not currently published.
    float confidence() const {
        if (_missCount <= HOLD_SAMPLES) return 1.0f;
        if (_missCount >= HOLD_SAMPLES + DECAY_SAMPLES) return 0.0f;
        uint16_t into = (uint16_t)(_missCount - HOLD_SAMPLES);
        return 1.0f - (float)into / (float)DECAY_SAMPLES;
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
};
