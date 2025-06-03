#!/usr/bin/env python3
#
# Automated tests for running 5G NR Sidelink SyncRef UE and/or Nearby UE.
# The following is an example to run remote machine (nearby) three times.
# The syncref UE is launched on the current machine, and a single
# nearby UE is launched on the machine specified by the --host and
# --user flags. The -r will enable this simulation to be repeated
# three times.
#
# python3 run_sl_test.py --user account --host 10.1.1.68 -r 3 --test usrp
#
# The following is an example to run just a Sidelink Nearby UE.
# By specifying -l nearby, only the nearby UE will be launched
# on the machine specified by the --host and --user flags.
# The -r will enable this simulation to be repeated two times.
#
# python3 run_sl_test.py -l nearby --user account --host 10.1.1.68 -r 2 --test usrp
#
# To run usrp test in the net 1 specified in sl_net_config.json, the following will be used.
#
# python3 run_sl_test.py --test usrp --net 1
#
# See `--help` for more information.
#

import os
import sys
import argparse
import logging
import time
import re
import glob
import json
import subprocess, shlex
from subprocess import Popen
from subprocess import check_output
from collections import defaultdict
import threading
from typing import Dict
from queue import Queue
from sl_check_log import LogChecker

HOME_DIR = os.path.expanduser( '~' )

# ----------------------------------------------------------------------------
# Command line argument parsing

parser = argparse.ArgumentParser(description="""
Automated tests for 5G NR Sidelink simulations
""")

parser.add_argument('--launch', '-l', default='all', choices='syncref nearby all'.split(), help="""
Sidelink UE type to launch test scenario (default: %(default)s)
""")

parser.add_argument('--host', default='', type=str, help="""
Nearby Host IP (default: %(default)s)
""")

parser.add_argument('--user', '-u', default=os.environ.get('USER'), type=str, help="""
User id in Nearby Host (default: %(default)s)
""")

parser.add_argument('--att', type=int, default=-1, help="""
Attenuation value
""")

parser.add_argument('--att-host', default='10.1.1.78', type=str, help="""
Host IP for adjusting attenuation (default: %(default)s)
""")

parser.add_argument('--att-user', default=os.environ.get('USER'), type=str, help="""
User id for adjusting attenuation (default: %(default)s)
""")

parser.add_argument('--basic', '-b', action='store_true', help="""
Basic test with basic shell commands
""")

parser.add_argument('--config', default='sl_net_config.json', help="""
The network configurations .json file (default: %(default)s)
""")

parser.add_argument('--compress', action='store_true', help="""
Compress the log files in the --log-dir
""")

parser.add_argument('--dest', type=str, default='', help="""
Destination node identifier for sidelink communication (default: %(default)s)
""")

parser.add_argument('--duration', '-d', metavar='SECONDS', type=int, default=20, help="""
How long to run the test before stopping to examine the logs
""")

parser.add_argument('--log-dir', default=HOME_DIR, help="""
Where to store log files
""")

parser.add_argument('--message', '-m', type=str, default='', help="""
The message to send from SyncRef UE to Nearby UE
""")

parser.add_argument('--mcs', default=0, type=int, help="""
The default mcs value (default: %(default)s)
""")

parser.add_argument('--net', '-n', type=int, default=1, help="""
Network identifier for sidelink communication (default: %(default)s)
""")

parser.add_argument('--nid1', type=int, default=10, help="""
Nid1 value
""")

parser.add_argument('--nid2', type=int, default=1, help="""
Nid2 value
""")

parser.add_argument('--no-run', action='store_true', help="""
Don't run the test, only examine the logs in the --log-dir
directory from a previous run of the test
""")

parser.add_argument('--debug', action='store_true', help="""
Enable debug logging (for this script only)
""")

parser.add_argument('--repeat', '-r', default=10, type=int, help="""

The number of repeated test iterations (default: %(default)s)
""")

parser.add_argument('--save', default=None, help="""
The default Python log result with .txt extension (default: %(default)s)
""")

parser.add_argument('--sci2', action='store_true', help="""
Enable SCI2 log parsing (this will grep the logs for the SCI2 payload)
""")

parser.add_argument('--test', '-t', default='rfsim', choices='rfsim usrp psbchsim psschsim'.split(), help="""
The kind of test scenario to run. The options include rfsim, usrp, psbchsim, and psschsim. (default: %(default)s)
""")

parser.add_argument('--snr', default='45', help="""

Setting snr values (default: %(default)s)
""")

OPTS = parser.parse_args()
del parser

logging.basicConfig(level=logging.DEBUG if OPTS.debug else logging.INFO,
                    format='>>> %(name)s: %(levelname)s: %(message)s')
LOGGER = logging.getLogger(os.path.basename(sys.argv[0]))

if OPTS.save:
    out_fh = logging.FileHandler(filename=OPTS.save, mode='a')
    LOGGER.addHandler(out_fh)
# ----------------------------------------------------------------------------
def redirect_output(cmd: str, filename: str) -> str:
    cmd += f' >{filename} 2>&1'
    return cmd

def thread_delay(delay: int) -> None:
    count = 0
    while count < 1:
        time.sleep(delay)
        count += 1

class Config:
    """
    Parsing config file
    """
    def __init__(self, filename) -> None:
        self.check_user()
        self.filename = self.check_file(filename)
        self.test = OPTS.test.lower()
        self.parse_config_json()

    def check_user(self) -> None:
        if 'usrp' != OPTS.test:
            return
        if OPTS.launch != 'syncref' and OPTS.user == '':
            LOGGER.error("--user followed by [user id] is mandatory to connect to remote machine")
            sys.exit(1)

    def check_file(self, filename) -> str:
        data_file = glob.glob(filename)
        if data_file:
            return filename
        LOGGER.error('The file %s does not exist!', filename)
        sys.exit(1)

    def parse_config_json(self) -> None:
        """
        Parse configurations from the json file
        """
        self.launch_cmds = defaultdict(list)
        self.hosts = defaultdict(list)

        with open(self.filename) as in_fh:
            json_data = json.load(in_fh)
            if f"net_{OPTS.net}" in json_data[self.test].keys():
                net_config = json_data[self.test][f"net_{OPTS.net}"]
            else:
                LOGGER.error('net %d configuration does not exist.', OPTS.net)
                sys.exit(1)
            for net in net_config:
                role = net['role']
                self.launch_cmds[role].append(net["cmd"])
                self.hosts[role].append(net["ip"])

        if not self.launch_cmds:
            LOGGER.error('%s commands are not found in file: %s', self.test, self.filename)
            sys.exit(1)
        if OPTS.test.lower() == 'usrp' and not self.hosts['nearby']:
            LOGGER.error(f'Nearby host IP is expected. Add nearby host IP to {OPTS.config}')
            sys.exit(1)

class Node:
    def __init__(self, id: str, role: str, host: str, cmd:str):
        self.id = id  # Assumption: UE node id starts from 1.
        self.role = role
        self.host = host
        self.delay = self._get_delay(OPTS.test)
        self.log_file_path = os.path.join(OPTS.log_dir, f'{role}{id}.log')
        syncref_node_id = '1' if id != '' else  '' # str(int(id) - 1)
        syncref_node_role = 'syncref' # if id in ('', '1', '2') else 'nearby' # Used for multi hop case.
        self.syncref_log_file_path = os.path.join(OPTS.log_dir, f'{syncref_node_role}{syncref_node_id}.log')
        self.cmd = self._update_cmd(role, cmd)
        self.passed_metric = []
        self.num_tx_ssb = []
        self.num_passed = 0
        self.total_rx_list = []
        self.nb_decoded_list = []
        self.pssch_rsrp_list = []
        self.ssb_rsrp_list = []
        self.sync_duration_list = []

    def __str__(self):
        return f'{self.role}{self.id}'

    def _get_delay(self, test_type: str) -> int:
        """
        Adjusting launching time by setting delay.
        """
        if test_type == 'usrp':
            return 5 if self.role == 'syncref' else (1 if self.id in ('', '1', '2') else 0)
        else:
            return 0 if self.role == 'syncref' else (2 if self.id in ('', '1', '2') else 7)

    def _update_cmd(self, role:str, cmd:str) -> str:
        if OPTS.basic: return redirect_output('uname -a', self.log_file_path)
        dest = '' if '--dest' in cmd or OPTS.dest == '' else f' --dest {OPTS.dest}'
        tx_msg = f" --message '{OPTS.message}'" if len(OPTS.message) > 0 else ""
        if role == 'syncref':
            cmd = cmd + tx_msg + f' --mcs {OPTS.mcs}' + dest
            if 'rfsim' == OPTS.test:
                cmd += f' --snr {OPTS.snr}'
        if role == 'nearby':
            if 'usrp' == OPTS.test:
                cmd = cmd[:-1] + tx_msg + f' --mcs {OPTS.mcs}' + cmd[-1] # -1 is ' index (end of cmd)
                cmd = cmd + f' -d {OPTS.duration} --nid1 {OPTS.nid1} --nid2 {OPTS.nid2}'
            else:
                cmd = cmd + tx_msg + f' --mcs {OPTS.mcs}'
                if 'rfsim' == OPTS.test:
                    cmd += f' --snr {OPTS.snr}'
        return cmd

    def get_metric(self, log_agent: LogChecker, itrn_inx: int) -> None:
        if self.num_passed != len(self.passed_metric):
            # Examine the logs to determine if the test passed
            (pssch_rsrp, ssb_rsrp, nb_decoded, total_rx, sync_duration) = self.passed_metric[-1]
            num_ssb = log_agent.analyze_syncref_logs(sync_duration, self.syncref_log_file_path)
            self.num_tx_ssb += [num_ssb]
            self.total_rx_list += [total_rx]
            self.sync_duration_list += [sync_duration]
            self.nb_decoded_list += [nb_decoded]
            self.pssch_rsrp_list += [pssch_rsrp]
            self.ssb_rsrp_list += [ssb_rsrp]
            LOGGER.info(f"Trial {itrn_inx + 1}/{OPTS.repeat} SYNCHED. {num_ssb} SSB(s) were generated. Measured {ssb_rsrp} RSRP (dbm/RE)")
        else:
            LOGGER.info(f"No metric available due to sync failure in {itrn_inx + 1}/{OPTS.repeat} trial(s).")
        self.num_passed = len(self.passed_metric)

    def show_metric(self) -> None:
        LOGGER.info('-' * 42)
        atten_snr = {"rfsim": f'SNR value {OPTS.snr}', "usrp": f'Attenuation value {OPTS.att}'}
        LOGGER.info(f"{atten_snr[OPTS.test]}, MCS value {OPTS.mcs}")

        LOGGER.info(f"Number of synced = {len(self.passed_metric)}/{OPTS.repeat}")
        if len(self.num_tx_ssb) > 0:
            LOGGER.info(f"Avg number of SSB = {sum(self.num_tx_ssb) / len(self.num_tx_ssb)} ({self.num_tx_ssb})")
        if len(self.passed_metric) > 0:
            sum_nb_decoded, sum_total_rx = sum(self.nb_decoded_list), sum(self.total_rx_list)
            avg_bler = (float) (sum_total_rx - sum_nb_decoded) / sum_total_rx if sum_total_rx > 0 else 1
            avg_bldr = (float) (sum_nb_decoded) / sum_total_rx if sum_total_rx > 0 else 1
            LOGGER.info(f"Avg PSSCH RSRP = {sum(self.pssch_rsrp_list) / len(self.passed_metric):.2f}")
            LOGGER.info(f"Avg SSB RSRP = {sum(self.ssb_rsrp_list) / len(self.passed_metric):.2f}")
            if sum_total_rx > 0:
                LOGGER.info(f"Avg BLER = {avg_bler:.9f} with {sum_total_rx - sum_nb_decoded} / {sum_total_rx}")
                LOGGER.info(f"Avg BLDecodedRate = {avg_bldr:.9f} with {sum_nb_decoded} / {sum_total_rx}")
            LOGGER.info(f"Avg Sync duration (seconds) = {sum(self.sync_duration_list) / len(self.passed_metric):.2f}")
            LOGGER.info(f"pssch_rsrp_list = {self.pssch_rsrp_list}")
            LOGGER.info(f"ssb_rsrp_list = {self.ssb_rsrp_list}")
            LOGGER.info(f"nb_decoded_list = {self.nb_decoded_list}")
            LOGGER.info(f"total_rx_list = {self.total_rx_list}")
        LOGGER.info('#' * 42)

class TestThread(threading.Thread):
    """
    Represents TestThread
    """
    def __init__(self, queue, log_agent):
        threading.Thread.__init__(self)
        self.queue = queue
        self.delay = 0
        self.log_agent = log_agent

    def run(self):
        if self.queue.empty():
            LOGGER.error("Queue is empty!")
            sys.exit(1)
        try:
            while not self.queue.empty():
                job = self.queue.get()
                if "nearby" == job.role and not OPTS.no_run:
                    thread_delay(self.delay + job.delay)
                    if 'usrp' == OPTS.test:
                        self.launch_nearby_usrp(job)
                    elif 'rfsim' == OPTS.test:
                        self.launch_nearby_rfsim(job)
                if "syncref" == job.role and not OPTS.no_run:
                    thread_delay(self.delay + job.delay)
                    self.launch_syncref(job)
            self.queue.task_done()
        except Exception as inst:
            LOGGER.info(f"Failed to operate on job with type {type(inst)} and args {inst.args}")

    def launch_syncref(self, job: Node) -> Popen:
        LOGGER.info(f'Launching SyncRef UE {job}')
        proc = Popen(job.cmd, shell=True)
        LOGGER.info(f"syncref_proc = {proc}")
        if not OPTS.basic and not OPTS.no_run:
            LOGGER.info(f"Process running... {job}")
            time.sleep(OPTS.duration + 10)
            self.kill_process(job, proc)

    def launch_nearby_usrp(self, job: Node) -> Popen:
        LOGGER.info(f'Launching Nearby UE {job}')
        proc = Popen(["ssh", f"{OPTS.user}@{job.host}", job.cmd],
                    shell=False,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE)
        LOGGER.info(f"nearby_proc = {proc}")
        remote_output = proc.stdout.readlines()
        if remote_output == []:
            nearby_result = proc.stderr.readlines()
        else:
            nearby_result = remote_output
        self.kill_process(job, proc)
        if nearby_result:
            self.find_nearby_result_metric(job, nearby_result)

    def launch_nearby_rfsim(self, job: Node) -> Popen:
        LOGGER.info(f'Launching Nearby UE {job}')
        proc = Popen(job.cmd, shell=True)
        LOGGER.info(f"nearby_proc = {proc}")
        if not OPTS.basic and not OPTS.no_run:
            LOGGER.info(f"Process running... {job}")
            time.sleep(OPTS.duration)
            self.kill_process(job, proc)
            if proc:
                time.sleep(5)
        nearby_result, user_msg = self.log_agent.analyze_nearby_logs(OPTS.nid1, OPTS.nid2, OPTS.sci2, job.log_file_path)
        if nearby_result:
            self.find_nearby_result_metric(job, [nearby_result])

    def find_nearby_result_metric(self, job: Node, remote_log) -> None:
        result_metric = None
        for line in remote_log:
            if type(line) is not str:
                line = line.decode()
            if 'usrp' == OPTS.test:
                LOGGER.info(line.strip())
            # 'SyncRef UE found. PSSCH-RSRP: -102 dBm/RE SSS-RSRP: -100 dBm/RE passed 99 total 100 It took {delta_time_s} seconds with data rate {data_rate} bit/s'
            if 'SyncRef UE found' in line:

                fields = line.split(maxsplit=25)
                if len(fields) > 6:

                    pssch_rsrp = float(fields[4])


                    ssb_rsrp = float(fields[7])
                    nb_decoded = int(fields[10])
                    total_rx = int(fields[12])
                    sync_duration = float(fields[15])
                    result_metric = (pssch_rsrp, ssb_rsrp, nb_decoded, total_rx, sync_duration)
                    job.passed_metric += [result_metric]
                    return

    def kill_process(self, job: Node, proc: Popen) -> None:
        # Wait for the processes to end
        passed = True
        if proc:
            status = proc.poll()
            if status is None:
                LOGGER.info('process is still running, which is good')
            else:
                passed = False
                LOGGER.info(f'{job} process ended early: {status}')
        if proc and passed:
            LOGGER.info(f'kill main simulation processes... {job}')
            cmd = ['sudo', 'killall']
            cmd.append('-KILL')
            cmd.append('nr-uesoftmodem')
            if "syncref" == job.role:
                subprocess.run(cmd)
            LOGGER.info(f'Waiting for PID proc.pid for {job}')
            proc.kill()
            proc.wait()
        LOGGER.info(f'kill main simulation processes...done for {job}')

# ----------------------------------------------------------------------------

def set_attenuation(attenuation: int, atten_host: str, user: str) -> None:
    """
    Attenuation value will be updated only if non-negative is specified by user.
    """
    if OPTS.att >= 0:
        LOGGER.info('Setting attenuation')
        atten_set_cmd = f"curl http://169.254.10.10/:CHAN:1:2:3:4:SETATT:{attenuation}" #CHAN:1:2:3:4:SETATT:25.5
        atten_get_cmd = f"curl http://169.254.10.10/:ATT?"
        host_IPs = check_output(['hostname', '-I']).decode().strip().split()
        LOCAL_IP = host_IPs[1] if len(host_IPs) > 1 else host_IPs[0]
        atten_cmds = [atten_set_cmd, atten_get_cmd]
        cmd = shlex.split('; '.join(atten_cmds)) if atten_host == LOCAL_IP else ["ssh", f"{user}@{atten_host}"] + atten_cmds
        proc = Popen(cmd,
                    shell=False,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE)
        out = proc.communicate(timeout=5)[0].strip().split('\n')[-1]
        LOGGER.info(f"attenuation value {attenuation} among: {out}")
        time.sleep(1)

def generate_jobs(config: Config) -> list:
    jobs, node_id, host = [], '', OPTS.host
    if config.launch_cmds is not None:
        for role, cmd_list in config.launch_cmds.items():
            for cmd in cmd_list:
                if 'node-number' in cmd:
                    node_id = cmd.split('node-number', 1)[-1].split(maxsplit=1)[0]
                if role == 'nearby' and OPTS.host == '':
                    host = config.hosts[role].pop(0)
                if OPTS.launch == 'all':
                    jobs.append(Node(node_id, role, host, cmd))
                elif role == OPTS.launch:
                    jobs.append(Node(node_id, role, host, cmd))
                LOGGER.debug(f'{role}{node_id} UE IP : {host}')
                LOGGER.debug(f'{role}{node_id} UE cmd: {cmd}')
    if OPTS.test == 'usrp': jobs.reverse()
    return jobs

def main() -> int:
    """
    Main function to run sidelink test repeatedly for a given attenuation value.
    """
    config = Config(OPTS.config)
    log_agent = LogChecker(OPTS, LOGGER)
    LOGGER.debug(f'Number of iterations {OPTS.repeat}')
    jobs = generate_jobs(config)
    if 'usrp' == OPTS.test:
        set_attenuation(OPTS.att, OPTS.att_host, OPTS.att_user)
    for i in range(OPTS.repeat):
        LOGGER.info('-' * 42)
        threads = []
        queue = Queue()
        for job in jobs:
            queue.put(job)
            th = TestThread(queue, log_agent)
            th.setDaemon(True)
            th.start()
            threads.append(th)
        for th in threads:
            th.join()
        for job in [job for job in jobs if job.role == 'nearby']:
            job.get_metric(log_agent, i)
            job.show_metric()
        time.sleep(10)
    return 0

sys.exit(main())
