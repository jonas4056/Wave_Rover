"""Toestand van de Jetson voor het dashboard: CPU, geheugen, GPU, temperatuur, vermogen,
ventilator, en CPU/geheugen per module.

Alles uit /proc en /sys (ook in de container leesbaar; /proc/stat en /proc/meminfo gelden voor
de hele Jetson). module_manager leest dit alleen zolang iemand /system/status volgt (paneel
Modules open in de app): een paar ms per keer, niets als niemand kijkt.
Processen buiten de container (camera-daemon van NVIDIA, Docker, Linux zelf) zitten in
"overig".
"""
import os

GPU_LOAD = '/sys/devices/platform/bus@0/17000000.gpu/load'  # 0-1000
PAGE_MB = os.sysconf('SC_PAGE_SIZE') / 1e6
TICKS = os.sysconf('SC_CLK_TCK')
CPUS = os.cpu_count() or 1


def read(path, default=None):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def find_hwmon(name):
    for entry in sorted(os.listdir('/sys/class/hwmon')):
        path = os.path.join('/sys/class/hwmon', entry)
        if read(os.path.join(path, 'name')) == name:
            return path
    return None


class SystemStatus:

    def __init__(self):
        self.ina = find_hwmon('ina3221')   # in1/curr1 = VDD_IN: wat de Jetson verbruikt
        self.fan = find_hwmon('pwm_tach')
        self.last_cpu = None               # (busy, total) jiffies van /proc/stat
        self.last_ticks = {}               # pgid of 'basis' -> (cpu-ticks, tijd)

    def cpu_percent(self):
        fields = [int(v) for v in read('/proc/stat', 'cpu 0').split('\n')[0].split()[1:]]
        idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
        busy, total = sum(fields) - idle, sum(fields)
        last, self.last_cpu = self.last_cpu, (busy, total)
        if last is None or total == last[1]:
            return None
        return round(100 * (busy - last[0]) / (total - last[1]))

    @staticmethod
    def memory_mb():
        info = {}
        for line in read('/proc/meminfo', '').split('\n'):
            key, _, value = line.partition(':')
            if value:
                info[key] = int(value.split()[0]) // 1024
        total = info.get('MemTotal', 0)
        return total - info.get('MemAvailable', total), total

    @staticmethod
    def temperature():
        temps = []
        for zone in os.listdir('/sys/class/thermal'):
            if not zone.startswith('thermal_zone'):
                continue
            kind = read(f'/sys/class/thermal/{zone}/type', '')
            value = read(f'/sys/class/thermal/{zone}/temp')
            if kind in ('cpu-thermal', 'gpu-thermal', 'tj-thermal') and value:
                temps.append(int(value) / 1000)
        return round(max(temps)) if temps else None

    def power_w(self):
        if not self.ina:
            return None
        mv, ma = read(f'{self.ina}/in1_input'), read(f'{self.ina}/curr1_input')
        return round(int(mv) * int(ma) / 1e6, 1) if mv and ma else None

    def processes(self):
        """{pgid: (rss MB, cpu-ticks)} van alle processen die we zien (de container)."""
        groups = {}
        for pid in os.listdir('/proc'):
            if not pid.isdigit():
                continue
            stat = read(f'/proc/{pid}/stat')
            statm = read(f'/proc/{pid}/statm')
            if not stat or not statm:
                continue
            fields = stat.rsplit(')', 1)[1].split()
            pgid, ticks = int(fields[2]), int(fields[11]) + int(fields[12])
            rss = int(statm.split()[1]) * PAGE_MB
            mb, t = groups.get(pgid, (0.0, 0))
            groups[pgid] = (mb + rss, t + ticks)
        return groups

    def usage(self, key, mb, ticks, now):
        """{'mb', 'cpu'} met cpu in % van de hele Jetson (alle kernen) sinds de vorige keer."""
        last = self.last_ticks.get(key)
        self.last_ticks[key] = (ticks, now)
        cpu = None
        if last and now > last[1]:
            cpu = round(100 * (ticks - last[0]) / TICKS / (now - last[1]) / CPUS, 1)
        return {'mb': round(mb), 'cpu': cpu}

    def read(self, module_groups, now):
        """module_groups: {modulenaam: pgid} van de draaiende modules."""
        used, total = self.memory_mb()
        groups = self.processes()
        modules = {}
        for name, pgid in module_groups.items():
            mb, ticks = groups.pop(pgid, (0.0, 0))
            modules[name] = self.usage(name, mb, ticks, now)
        base_mb = sum(mb for mb, _ in groups.values())
        base = self.usage('basis', base_mb, sum(t for _, t in groups.values()), now)
        fan = read(f'{self.fan}/rpm') if self.fan else None
        return {
            'cpu': self.cpu_percent(),
            'ram_used': used,
            'ram_total': total,
            'gpu': round(int(read(GPU_LOAD, '0')) / 10),
            'temp': self.temperature(),
            'power': self.power_w(),
            'fan': int(fan) if fan else None,
            'modules': modules,
            'basis': base,
            'overig_mb': max(0, round(used - base_mb - sum(m['mb'] for m in modules.values()))),
        }
