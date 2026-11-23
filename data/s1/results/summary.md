Clock alignment: knock (start+end). Trials analysed: 16. Robot yaw rate from lowstate (low.csv gz) in 16 trials.

| stick | profile | n | hand jerk added (m/s³) | peak hand accel (m/s²) | hand lag (s) | hand cue onset (s) | robot lag (s) | joint work above walking (J) | turn (deg) |
|---|---|---|---|---|---|---|---|---|---|
| rigid | smooth | 8 | 6.26 [0.78–10.16] | 3.38 [3.36–3.70] | 0.28 [0.23–0.33] | 0.36 [0.15–0.42] | 0.15 [0.14–0.18] | 4.04 [-2.94–18.75] | 79 |
| rigid | step | 8 | 6.89 [1.06–12.18] | 3.42 [3.14–3.80] | 0.27 [0.22–0.29] | 0.19 [0.06–0.37] | 0.16 [0.13–0.18] | 9.05 [5.39–12.32] | 79 |

Median [IQR]. n = trials per condition ('hand k': only k of them have phone data), one participant (the author), sighted, eyes open. Exploratory pilot, no inferential statistics.
Lags: difference of the signal centroids over [turn start - 0.5 s, turn end + 1.5 s] (robot vs command, hand vs robot), on baseline-removed yaw rates signed so the turn is positive; for a linear response they do not depend on the command shape, so hand lag is the delay added by the stick, not when the hand first feels the turn. Hand cue onset: time from the turn command until the hand yaw rate first holds >= 0.1 rad/s for 0.25 s; it does depend on the command shape. 30%-of-peak onset lags are in trials.csv (robot_onset_lag_s, hand_onset_lag_s). Jerk window: 4.12 s from turn start for both profiles. Joint work above walking: integral of (power - mean power in the 2 s before the turn) over turn + 1.5 s; the absolute integrals (mech_work_J, elec_energy_J in trials.csv) grow with the window, which is 0.8 s longer for smooth. Turn: signed integral of the yaw rate, minus the gyro bias of the 1 s 'still' phase only if the robot was still there (yaw-rate std < 0.02 rad/s): 16 of 16 trials (gyro_bias, gyro_still_std in trials.csv).
Axis signs from the data: robot gyro z vs command +1 (16/16 trials agree), hand yaw vs robot yaw -1 (16/16).

## Profile x direction

| stick | profile | L | R |
|---|---|---|---|
| rigid | step | 4 | 4 |
| rigid | smooth | 4 | 4 |

Step - smooth difference of the medians, per turn direction and averaged over the directions (the averaged value is not biased by an unequal number of left and right turns per profile; cells hold only a few trials, so this is descriptive):

| stick | metric | pooled | L | R | mean over directions |
|---|---|---|---|---|---|
| rigid | hand jerk added (m/s³) | 0.63 | 1.10 | 4.04 | 2.57 |
| rigid | peak hand accel (m/s²) | 0.04 | -0.10 | -0.26 | -0.18 |
| rigid | hand lag (s) | -0.01 | -0.06 | 0.03 | -0.02 |
| rigid | hand cue onset (s) | -0.17 | -0.34 | 0.11 | -0.11 |
| rigid | robot lag (s) | 0.01 | 0.04 | -0.00 | 0.02 |
| rigid | joint work above walking (J) | 5.01 | -6.03 | 5.70 | -0.16 |
| rigid | battery energy above walking (J) | -7.39 | -20.92 | -0.66 | -10.79 |
| rigid | turn (deg) | -0.62 | -2.29 | 2.15 | -0.07 |

## Skipped trials

None: every row of trials.csv was analysed.

## T6 checks

- PASS sync_start: 5 knocks matched (>= 4 needed)
- PASS sync_end: 5 knocks matched (>= 4 needed); clock drift 69.6 ms
- WARN robot_turn_deg outside 80-100 deg: trial 2: 70.14, trial 4: 62.89, trial 6: 69.79, trial 8: 69.84, trial 10: 64.38, trial 12: 69.30, trial 14: 64.02, trial 16: 69.18
- PASS hand_lag_s within 0-1 s in all 16 analysed trials
- PASS no aborted, skipped or excluded trials

## Clock sync

Model: knock (start+end), drift 69.6 ms between the syncs.
- sync_start: 5 robot knocks, 5 matched, offset 13.9228 s; robot top peaks [1.99, 1.37, 1.28, 1.12, 0.92] (threshold 0.5 m/s²); phone top peaks [6.31, 5.49, 3.66, 3.64, 3.31]; candidates (matched, offset s, extra peaks) [(5, 13.9228, 0), (5, 612.4823, 9), (5, 396.9925, 16), (4, 554.7964, 10)]
- sync_end: 5 robot knocks, 5 matched, offset 13.9924 s; robot top peaks [1.49, 1.45, 1.39, 1.37, 1.1] (threshold 0.5 m/s²); phone top peaks [3.75, 3.64, 3.45, 2.39, 2.26]; candidates (matched, offset s, extra peaks) [(5, 13.9924, 0), (5, -737.8528, 7), (5, -819.5229, 11), (5, -818.6911, 11)]
