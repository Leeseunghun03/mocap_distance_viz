# -*- coding: utf-8 -*-
"""Terminal dashboard for the distance visualizer (no ROS dependency)."""

from __future__ import division

import collections
import os
import struct
import sys
import time

try:
    import fcntl
    import termios
except ImportError:  # non-POSIX
    fcntl = None

PY2 = sys.version_info[0] == 2
if PY2:
    text_type = unicode  # noqa: F821
else:
    text_type = str

ESC = u'\033['
RESET = ESC + u'0m'
BOLD = ESC + u'1m'
DIM = ESC + u'2m'
GREEN = ESC + u'32m'
YELLOW = ESC + u'33m'
RED = ESC + u'31m'
CYAN = ESC + u'36m'
CLEAR_LINE = ESC + u'2K'

H_LINE = u'─'
DOT = u'●'
MIDDOT = u'·'

SPEED_WINDOW = 1.0  # [s]
WIDE = 84           # min. columns for the POSITION column


def is_tty(stream=None):
    stream = stream or sys.stdout
    try:
        return stream.isatty()
    except (AttributeError, ValueError):
        return False


def terminal_width(stream=None, default=100):
    stream = stream or sys.stdout
    if fcntl is not None:
        try:
            rows_cols = fcntl.ioctl(stream.fileno(), termios.TIOCGWINSZ, b'\0' * 4)
            cols = struct.unpack('hh', rows_cols)[1]
            if cols > 0:
                return cols
        except (IOError, OSError, ValueError, AttributeError):
            pass
    try:
        return int(os.environ.get('COLUMNS', default))
    except ValueError:
        return default


def _z(v):
    """Avoid printing '-0.000'."""
    return 0.0 if abs(v) < 0.0005 else v


def fmt_hms(seconds):
    seconds = int(max(0, seconds))
    return u'%02d:%02d:%02d' % (seconds // 3600, seconds // 60 % 60, seconds % 60)


class BodyRow(object):
    """Snapshot of one body for rendering."""

    __slots__ = ('name', 'color', 'has_data', 'distance', 'displacement',
                 'position', 'age', 'jumps')

    def __init__(self, name, color, has_data, distance=0.0, displacement=0.0,
                 position=None, age=None, jumps=0):
        self.name = name
        self.color = color          # (r, g, b) in 0..1
        self.has_data = has_data
        self.distance = distance
        self.displacement = displacement
        self.position = position    # (x, y, z) or None
        self.age = age              # seconds since last sample, None if never
        self.jumps = jumps


class Dashboard(object):

    def __init__(self, title, reset_hint, stale_timeout=0.5, color=True, stream=None):
        self.title = title
        self.reset_hint = reset_hint
        self.stale_timeout = stale_timeout
        self.color = color and 'NO_COLOR' not in os.environ
        self.truecolor = os.environ.get('COLORTERM', '') in ('truecolor', '24bit')
        self.stream = stream or sys.stdout
        self.drawn_lines = 0
        self.history = {}
        self.start = time.time()
        self.event = None   # (wall time, text)

    def reset(self, now=None):
        now = time.time() if now is None else now
        self.history = {}
        self.start = now
        self.event = (now, u'reset')

    def speed(self, name, distance, now):
        """Average speed over the last SPEED_WINDOW seconds, None if unknown."""
        hist = self.history.setdefault(name, collections.deque())
        hist.append((now, distance))
        while len(hist) > 2 and now - hist[1][0] >= SPEED_WINDOW:
            hist.popleft()
        t0, d0 = hist[0]
        if now - t0 < 0.5 * SPEED_WINDOW:
            return None
        return (distance - d0) / (now - t0)

    def _c(self, code, text):
        return code + text + RESET if self.color else text

    def _body_color(self, rgb):
        if not self.color:
            return u''
        r, g, b = rgb
        if self.truecolor:
            return ESC + u'38;2;%d;%d;%dm' % (int(r * 255), int(g * 255), int(b * 255))
        idx = 16 + 36 * int(round(r * 5)) + 6 * int(round(g * 5)) + int(round(b * 5))
        return ESC + u'38;5;%dm' % idx

    def render(self, rows, now=None, width=None):
        """Return the dashboard as a list of lines (without newlines)."""
        now = time.time() if now is None else now
        width = width or terminal_width(self.stream)
        wide = width >= WIDE
        rule_w = min(width, WIDE if wide else 58) - 2
        rule = u'  ' + self._c(DIM, H_LINE * rule_w)

        elapsed = u'elapsed ' + fmt_hms(now - self.start)
        title = u' ' + self.title
        pad = max(1, rule_w + 2 - len(title) - len(elapsed))
        lines = [u'', self._c(BOLD + CYAN, title) + u' ' * pad + self._c(DIM, elapsed), rule]

        head = u'    %-12s %9s %9s %11s' % (u'BODY', u'PATH m', u'DISP m', u'SPEED m/s')
        if wide:
            head += u'   %-24s' % u'POSITION x, y, z m'
        head += u'  STATUS'
        lines.append(self._c(DIM, head))

        for row in rows:
            lines.append(self._render_row(row, now, wide))

        lines.append(rule)
        footer = u'    reset: ' + self.reset_hint
        if self.event is not None:
            ev = u'%s %s' % (time.strftime('%H:%M:%S', time.localtime(self.event[0])), self.event[1])
            pad = max(2, rule_w + 2 - len(footer) - len(ev))
            lines.append(self._c(DIM, footer) + u' ' * pad + self._c(YELLOW, ev))
        else:
            lines.append(self._c(DIM, footer))
        return lines

    def _render_row(self, row, now, wide):
        name = row.name if len(row.name) <= 12 else row.name[:11] + u'~'
        dot = self._body_color(row.color) + DOT + (RESET if self.color else u'')
        line = u'  ' + dot + u' ' + self._c(BOLD, u'%-12s' % name)

        if not row.has_data:
            line += u' %9s %9s %11s' % (u'-', u'-', u'-')
            if wide:
                line += u'   %-24s' % u''
            return line + u'  ' + self._c(DIM, u'waiting')

        spd = self.speed(row.name, row.distance, now)
        spd_txt = u'%11s' % (u'-' if spd is None else u'%.2f' % spd)
        line += u' ' + self._c(BOLD, u'%9.3f' % row.distance)
        line += u' %9.3f' % row.displacement
        line += u' ' + self._c(DIM, spd_txt)
        if wide:
            x, y, z = row.position
            line += u'   ' + self._c(DIM, u'(%6.3f, %6.3f, %6.3f)' % (_z(x), _z(y), _z(z)))

        if row.age is not None and row.age > self.stale_timeout:
            status = self._c(YELLOW, u'lost %.1fs' % row.age)
        else:
            status = self._c(GREEN, u'tracking')
        if row.jumps:
            status += self._c(DIM, u' %s ' % MIDDOT) + self._c(YELLOW, u'%d jump%s' % (
                row.jumps, u'' if row.jumps == 1 else u's'))
        return line + u'  ' + status

    def draw(self, rows, now=None):
        """Redraw the dashboard in place."""
        lines = self.render(rows, now)
        out = u''
        if self.drawn_lines:
            out += ESC + u'%dF' % self.drawn_lines
        out += u''.join(CLEAR_LINE + line + u'\n' for line in lines)
        if len(lines) < self.drawn_lines:
            out += (CLEAR_LINE + u'\n') * (self.drawn_lines - len(lines))
            out += ESC + u'%dF' % (self.drawn_lines - len(lines))
        self.drawn_lines = len(lines)
        self._write(out)

    def _write(self, text):
        data = text.encode('utf-8') if isinstance(text, text_type) else text
        if PY2:
            self.stream.write(data)
        else:
            getattr(self.stream, 'buffer', self.stream).write(data)
        self.stream.flush()


def summary_line(rows, dashboard, now=None):
    """Plain one-line summary for non-TTY output (log files, pipes)."""
    now = time.time() if now is None else now
    parts = []
    for row in rows:
        if not row.has_data:
            parts.append('%s: waiting' % row.name)
            continue
        spd = dashboard.speed(row.name, row.distance, now)
        part = '%s: path=%.3f m disp=%.3f m' % (row.name, row.distance, row.displacement)
        if spd is not None:
            part += ' speed=%.2f m/s' % spd
        if row.age is not None and row.age > dashboard.stale_timeout:
            part += ' LOST %.1fs' % row.age
        parts.append(part)
    return ' | '.join(parts)
