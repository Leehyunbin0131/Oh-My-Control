#!/usr/bin/env python3
"""Disarmed PING benchmark. Does not validate active PID or SLAM performance."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import time

from motor_test import MotorClient, MotorError, DEFAULT_MOTOR_PORT, frame

ROOT = Path(__file__).resolve().parents[1]

class ProbeClient(MotorClient):
    def send(self, operation, *values):
        if operation not in ('STOP', 'PING') or values:
            raise MotorError('This profiler permits only STOP and PING')
        return super().send(operation, *values)

def check_disabled(client):
    sample = client.latest
    if sample['fault'] or sample['armed'] or any(sample['command_pwm']):
        raise MotorError('Fault or enabled motor output during disarmed profiling')

def percentile(values, fraction):
    return sorted(values)[max(0, math.ceil(len(values)*fraction)-1)]

def measure(client, hz, seconds):
    client.log.flush()
    initial_bytes = client.log.tell()
    initial_sample = client.latest
    initial_timing = dict(client.timing)
    sent, rtts, samples, tx_bytes = [], [], [], 0
    began = next_send = time.monotonic()
    last_sample = client.latest
    while time.monotonic()-began < seconds:
        now = time.monotonic()
        if now >= next_send:
            before = time.monotonic()
            seq = client.command('PING')
            after = time.monotonic()
            sent.append(before)
            rtts.append((after-before)*1000)
            tx_bytes += len(frame(f'OMR1,C,{seq},PING'))
            # Never send a catch-up burst after a scheduling delay.
            next_send = max(before+1/hz, after)
        else:
            client.poll()
        check_disabled(client)
        if client.latest is not last_sample:
            samples.append(client.latest)
            last_sample = client.latest
    elapsed = time.monotonic()-began
    client.log.flush()
    gaps = [(b-a)*1000 for a,b in zip(sent,sent[1:])]
    telemetry_times = [s['received_at'] for s in samples]
    rx_bytes = client.log.tell()-initial_bytes
    errors = client.receive_diagnostic
    missed = client.timing['missed_ticks']-initial_timing['missed_ticks']
    invalid = [(a-b)&0xffffffff for a,b in zip(client.latest['invalid'],initial_sample['invalid'])]
    ok = (not client.decoder.errors and not missed and not any(invalid) and
          all(errors[k]==0 for k in ('frame_errors','crc_errors','sequence_errors')))
    return {'requested_hz':hz,'status':'PASS' if ok else 'FAIL','duration_s':elapsed,
            'acknowledged':len(sent),'actual_hz':(len(sent)-1)/(sent[-1]-sent[0]),
            'rtt_ms':{'median':statistics.median(rtts),'p95':percentile(rtts,.95),'max':max(rtts)},
            'send_gap_ms':{'p95':percentile(gaps,.95),'max':max(gaps)},
            'telemetry_hz':(len(samples)-1)/(telemetry_times[-1]-telemetry_times[0]),
            'uart_tx_utilization_percent':tx_bytes*10/115200/elapsed*100,
            'uart_rx_utilization_percent':rx_bytes*10/115200/elapsed*100,
            'missed_control_ticks_delta':missed,'encoder_invalid_delta':invalid,
            'receive':errors,'timing':client.timing,'host_crc_errors':client.decoder.errors}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=DEFAULT_MOTOR_PORT)
    parser.add_argument('--seconds', type=float, default=8)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 3 <= args.seconds <= 30:
        parser.error('--seconds must be between 3 and 30')
    folder = ROOT/'reports'/('motor_link_profile_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir()
    result = {'status':'FAIL','scope':'Disarmed PING/ACK, no active PID or SLAM load',
              'motor_driven':False,'rates':[],'error':None,
              'limitations':['PING frames are shorter than wheel-speed commands',
                             'Idle timing does not measure active PID execution cost',
                             'Nav2 and SLAM are not running during this check']}
    try:
        with ProbeClient(args.port,folder) as client:
            client.collect(1.2)
            check_disabled(client)
            client.serial.timeout=.001
            result['firmware']=client.info
            for hz in (20,50,100):
                trial=measure(client,hz,args.seconds)
                result['rates'].append(trial)
                print(json.dumps(trial,ensure_ascii=False),flush=True)
                if trial['status'] != 'PASS':
                    raise MotorError('Timing or communication error; profiling stopped')
            result['final_sample']=client.latest
            result['status']='PASS'
    except Exception as exc:
        result['error']=f'{type(exc).__name__}: {exc}'
    finally:
        (folder/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print('REPORT',folder/'summary.json',flush=True)
    return 0 if result['status']=='PASS' else 1

if __name__=='__main__':
    raise SystemExit(main())
