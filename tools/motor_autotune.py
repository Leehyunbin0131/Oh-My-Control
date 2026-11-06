"""Bounded on-hardware PID gain search using repeated measured step responses.

Feedforward, direction mapping, PWM ceiling and MCU guards stay fixed. Search
uses 30/72 RPM starts; interleaved original/candidate trials at 50/90 RPM are
held out for confirmation. This optimizes unloaded starts, not ground traction
or disturbance rejection under a physical load.
"""

import copy
import json
import math
from pathlib import Path
import statistics
import time

from motor_pid import PIDSettings
from motor_test import MotorError

WHEELS = ('left', 'right')
BOUNDS = {'kp': (0.25, 5.0), 'ki': (2.0, 25.0), 'kd': (0.0, 0.06)}
SEARCH_RPM = (30, 72)
CONFIRM_RPM = (50, 90)


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def response_metrics(points, target):
    """Score physical-wheel (elapsed seconds, signed RPM, signed PWM) samples.

    All candidates use the same duration. Score is normalized absolute error
    integral + overshoot + steady RMS + settling time + steady PWM variation.
    Time starts immediately before transmitting SPEED, including the real start
    delay. No first nonzero sample alignment hides a slow start.
    """
    if not math.isfinite(target) or not 15 <= abs(target) <= 100 or len(points) < 30:
        raise ValueError('오토튠 목표/응답 샘플 부족')
    if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('오토튠 응답에 유효하지 않은 값이 있습니다.')
    times = [p[0] for p in points]
    if times[0] < 0 or times[0] > 0.15 or times[-1] < 2.2 or any(
            not 0 < b - a <= 0.15 for a, b in zip(times, times[1:])):
        raise ValueError('오토튠 응답 시간/샘플 간격 오류')
    sign = 1 if target > 0 else -1
    goal = abs(target)
    speeds = [p[1] * sign for p in points]
    errors = [goal - value for value in speeds]
    tail = [p for p in points if p[0] >= times[-1] - 0.7]
    tail_errors = [p[1] - target for p in tail]
    rms = math.sqrt(statistics.mean(e * e for e in tail_errors))
    overshoot = max(0.0, max(speeds) - goal)
    tolerance = max(1.0, goal * 0.02)
    last_outside = max((i for i, error in enumerate(errors) if abs(error) > tolerance), default=-1)
    settled = last_outside < len(points) - 1
    settling = times[last_outside + 1] if settled else times[-1]
    rise90 = next((t for t, speed in zip(times, speeds) if speed >= goal * 0.9), times[-1])
    # Trapezoidal integration includes the initial zero-speed condition.
    iae, previous_time, previous_error = 0.0, 0.0, goal
    for stamp, error in zip(times, errors):
        iae += (stamp - previous_time) * (abs(error) + previous_error) / 2
        previous_time, previous_error = stamp, abs(error)
    pwm_variation = statistics.mean(abs(b[2] - a[2]) for a, b in zip(tail, tail[1:]))
    mean_rpm = statistics.mean(p[1] for p in tail)
    score = iae / goal + 1.5 * overshoot / goal + 2.0 * rms / goal + 0.15 * settling + 0.1 * pwm_variation / 650
    acceptable = (overshoot <= max(5, 0.15 * goal) and rms <= max(1.2, goal * 0.035) and
                  abs(mean_rpm - target) <= max(1, goal * 0.025) and min(speeds) >= -2 and
                  max(abs(p[2]) for p in points) <= 650)
    return {'score': score, 'acceptable': acceptable, 'target_rpm': target, 'mean_rpm': mean_rpm,
            'iae_rpm_seconds': iae, 'normalized_iae_seconds': iae / goal, 'overshoot_rpm': overshoot,
            'steady_rms_error_rpm': rms, 'settling_seconds': settling, 'settled_at_end': settled,
            'rise90_seconds': rise90, 'pwm_variation': pwm_variation, 'samples': len(points)}


def aggregate(trials):
    return {name: {'score': statistics.mean(t['metrics'][name]['score'] for t in trials),
                   'acceptable': all(t['metrics'][name]['acceptable'] for t in trials),
                   'steady_rms_error_rpm': statistics.mean(t['metrics'][name]['steady_rms_error_rpm'] for t in trials),
                   'overshoot_rpm': statistics.mean(t['metrics'][name]['overshoot_rpm'] for t in trials),
                   'settling_seconds': statistics.mean(t['metrics'][name]['settling_seconds'] for t in trials),
                   'normalized_iae_seconds': statistics.mean(t['metrics'][name]['normalized_iae_seconds'] for t in trials)}
            for name in WHEELS}


def updated(document, direction, key, factor=None, value=None):
    result = copy.deepcopy(document)
    low, high = BOUNDS[key]
    for name in WHEELS:
        gains = result['wheels'][name][direction]
        gains[key] = round(max(low, min(high, value if value is not None else gains[key] * factor)), 3)
    return result


def select_direction(anchor, direction, evaluations, improvement=0.03):
    """Select each wheel independently, always copying a measured gain tuple."""
    selected = copy.deepcopy(anchor)
    winners = {}
    for name in WHEELS:
        reference = evaluations[0]
        winner = reference
        for item in evaluations[1:]:
            metric = item['aggregate'][name]
            if metric['acceptable'] and (not winner['aggregate'][name]['acceptable'] or
                                        metric['score'] < winner['aggregate'][name]['score'] * (1 - improvement)):
                winner = item
        selected['wheels'][name][direction] = copy.deepcopy(winner['document']['wheels'][name][direction])
        winners[name] = {'evaluation': winner['label'], 'score': winner['aggregate'][name]['score']}
    return selected, winners


def search(document, evaluate, progress=lambda stage, value: None):
    """Adaptive pattern search. evaluate always runs real motor experiments."""
    best = copy.deepcopy(document)
    stages = []
    for direction, sign in (('forward', 1), ('reverse', -1)):
        for level, factor in enumerate((2.0, 1.4), 1):
            anchor = copy.deepcopy(best)
            prefix = f'{direction}_round{level}'
            trials = [evaluate(anchor, sign, prefix + '_anchor', SEARCH_RPM)]
            for key in ('kp', 'ki'):
                for multiplier, suffix in ((1 / factor, 'down'), (factor, 'up')):
                    candidate = updated(anchor, direction, key, factor=multiplier)
                    trials.append(evaluate(candidate, sign, f'{prefix}_{key}_{suffix}', SEARCH_RPM))
            best, winners = select_direction(anchor, direction, trials)
            stages.append({'stage': prefix, 'winners': winners})
            progress(prefix, {'stages': stages, 'candidate': best})
        # Derivative is actually tested, and retained only if it improves score.
        anchor = copy.deepcopy(best)
        trials = [evaluate(anchor, sign, direction + '_derivative_anchor', SEARCH_RPM)]
        for kd in (0.0, 0.015, 0.03, 0.06):
            candidate = updated(anchor, direction, 'kd', value=kd)
            if all(candidate['wheels'][n][direction] == anchor['wheels'][n][direction] for n in WHEELS):
                continue
            trials.append(evaluate(candidate, sign, f'{direction}_kd_{kd:.3f}', SEARCH_RPM))
        best, winners = select_direction(anchor, direction, trials)
        stages.append({'stage': direction + '_derivative', 'winners': winners})
        progress(direction + '_derivative', {'stages': stages, 'candidate': best})
    return best, stages


def confirm(original, candidate, evaluate):
    """Independent speeds, two interleaved repetitions (A/B/B/A) per direction."""
    selected = copy.deepcopy(original)
    results = {}
    for direction, sign in (('forward', 1), ('reverse', -1)):
        baseline, tuned = [], []
        for index, label in enumerate(('original', 'candidate', 'candidate', 'original')):
            doc = original if label == 'original' else candidate
            trial = evaluate(doc, sign, f'confirm_{direction}_{index}_{label}', CONFIRM_RPM)
            (baseline if label == 'original' else tuned).append(trial)
        results[direction] = {}
        for name in WHEELS:
            old_score = statistics.mean(t['aggregate'][name]['score'] for t in baseline)
            new_score = statistics.mean(t['aggregate'][name]['score'] for t in tuned)
            old_rms = statistics.mean(t['aggregate'][name]['steady_rms_error_rpm'] for t in baseline)
            new_rms = statistics.mean(t['aggregate'][name]['steady_rms_error_rpm'] for t in tuned)
            accepted = (all(t['aggregate'][name]['acceptable'] for t in tuned) and
                        new_score <= old_score * 0.97 and new_rms <= max(0.7, old_rms * 1.25))
            if accepted:
                selected['wheels'][name][direction] = copy.deepcopy(candidate['wheels'][name][direction])
            results[direction][name] = {'accepted': accepted, 'baseline_score': old_score,
                                       'candidate_score': new_score, 'improvement_percent': (1 - new_score / old_score) * 100,
                                       'baseline': [t['aggregate'][name] for t in baseline],
                                       'candidate': [t['aggregate'][name] for t in tuned]}
    return selected, results


def rollback_failed_channels(candidate, original, failed_trial):
    """Final motion validation can veto a locally better transient candidate."""
    selected = copy.deepcopy(candidate)
    failed = []
    for name in WHEELS:
        metric = failed_trial['metrics'][name]
        goal = abs(metric['target_rpm'])
        if abs(metric['mean_error_rpm']) > max(1, goal * .015) or metric['rms_error_rpm'] > max(2, goal * .05):
            failed.append(name)
    if not failed and failed_trial['tracking_difference_rpm'] > 1.5:
        failed = list(WHEELS)
    rejections = []
    for name in failed:
        target = failed_trial['metrics'][name]['target_rpm']
        direction = 'forward' if target > 0 else 'reverse'
        if selected['wheels'][name][direction] == original['wheels'][name][direction]:
            continue
        selected['wheels'][name][direction] = copy.deepcopy(original['wheels'][name][direction])
        reason = {'wheel': name, 'direction': direction, 'target_rpm': failed_trial['target_rpm'],
                  'metrics': failed_trial['metrics'][name], 'reason': 'final motion validation failed'}
        rejections.append(reason)
        selected['autotune']['comparison'][direction][name].update(accepted=False, final_validation_rejection=reason)
    if not rejections:
        raise MotorError('기존 계수에서도 최종 검증 오류가 발생했습니다. 보정 파일을 유지합니다.')
    selected['autotune'].setdefault('final_validation_rejections', []).extend(rejections)
    return selected


class Experiment:
    def __init__(self, client, report, stop):
        self.client = client
        self.report = Path(report)
        self.stop = stop
        self.history = []

    def run(self, document, sign, label, rpms):
        trial_doc = copy.deepcopy(document)
        if 'hold' in trial_doc:
            trial_doc['hold']['enabled'] = False
        settings = PIDSettings(trial_doc, self.client.calibration, require_validated=False)
        folder = self.report / 'experiments' / f'{len(self.history):03d}_{label}'
        folder.mkdir(parents=True)
        save(folder / 'settings.json', document['wheels'])
        settings.configure(self.client)
        trials = []
        for rpm in rpms:
            target = sign * rpm
            raw = settings.commands(round(target * 100), round(target * 100))
            invalid = self.client.latest['invalid'][:]
            samples = []
            try:
                self.client.command('ARM')
                start = time.monotonic()
                self.client.set_speed(*raw, wait=True)

                def check(sample):
                    if sample['invalid'] != invalid:
                        raise MotorError('오토튠 중 엔코더 전이 오류')
                    if time.monotonic() - start > 0.15 and (not sample.get('pid_mode') or sample.get('target_rpm_centi') != raw):
                        raise MotorError('오토튠 중 목표 속도/구동 모드 불일치')
                    if any(abs(v['rpm']) > 100 for v in settings.wheel_speeds(sample).values()):
                        raise MotorError('오토튠 속도 상한 초과')

                self.client.collect(2.6, target=raw, sample_check=check)
                end = time.monotonic()
                samples = [s for s in self.client.samples if start <= s['received_at'] <= end and
                           s.get('pid_mode') and s.get('target_rpm_centi') == raw]
            finally:
                stop_samples = self.stop(self.client)
            points = {name: [(s['received_at'] - start, settings.wheel_speeds(s)[name]['rpm'],
                              settings.wheel_speeds(s)[name]['pwm']) for s in samples] for name in WHEELS}
            metrics = {name: response_metrics(points[name], target) for name in WHEELS}
            trial = {'target_rpm': target, 'metrics': metrics, 'points': points,
                     'samples': samples, 'stopped': stop_samples,
                     'braking': getattr(self.client, 'last_stop_report', None)}
            trials.append(trial)
            save(folder / 'trials.json', trials)
        result = {'label': label, 'document': copy.deepcopy(document), 'aggregate': aggregate(trials),
                  'directory': str(folder), 'targets': [sign * rpm for rpm in rpms]}
        self.history.append(result)
        save(self.report / 'search_history.json', self.history)
        print(f'AUTO {label}: ' + ' '.join(f'{n}={result["aggregate"][n]["score"]:.4f}' +
              ('' if result['aggregate'][n]['acceptable'] else ' (rejected)') for n in WHEELS), flush=True)
        return result


def autotune(client, original, report, stop):
    PIDSettings(original, client.calibration)
    for wheel in original['wheels'].values():
        for gains in wheel.values():
            if any(not low <= gains[key] <= high for key, (low, high) in BOUNDS.items()):
                raise ValueError('기존 PID 계수가 제한된 자동 탐색 범위를 벗어납니다.')
    experiment = Experiment(client, report, stop)
    save(Path(report) / 'autotune_original.json', original)
    candidate, stages = search(original, experiment.run,
                               lambda stage, value: save(Path(report) / 'search_progress.json', value))
    save(Path(report) / 'search_candidate.json', candidate)
    selected, comparisons = confirm(original, candidate, experiment.run)
    save(Path(report) / 'confirmation.json', comparisons)
    selected.update(validated=False, method='On-hardware bounded PID pattern search with fixed measured feedforward',
                    evidence_directory=str(report), additional_evidence_directories=[],
                    autotune={'completed': True, 'search_bounds': BOUNDS, 'search_rpm': SEARCH_RPM,
                              'confirmation_rpm': CONFIRM_RPM, 'confirmation_repetitions': 2,
                              'selection_threshold_percent': 3, 'experiment_batches': len(experiment.history),
                              'stages': stages, 'comparison': comparisons,
                              'scope': 'Unloaded starts; no physical load disturbance or traction identification'})
    selected.pop('validated_at', None)
    selected.pop('validation', None)
    return selected
