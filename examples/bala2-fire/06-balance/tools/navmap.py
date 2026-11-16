"""The map and the route finding of explorer.py. No robot here: only numbers in, numbers out.

The map is a grid of square cells. Each cell has a score: high means "something
is there", low means "free", near 0 means "not seen yet". It is a dict
{"ix,iy": score}, so it goes to JSON as it is.

The distance sensor does not see a line, it sees a cone (27 degrees wide with
the wide zone) and gives the nearest thing in it. So one reading says two
things: every cell of the cone that is nearer than the reading is free, and
something sits somewhere on the arc at that distance. One reading cannot say
where on the arc. A turn of the robot can: the cones of the next readings pass
over the arc, and they clear the cells where nothing is.

Python 3 standard library only.
"""
import heapq
import math

TOP = 6     # scores stay in -TOP .. TOP
OCC = 2     # a score from this on: something is there
FREE = -1   # a score down to this: free


def norm(deg):
    """An angle in -180 .. 180."""
    return (deg + 180) % 360 - 180


def cell_of(cell, x, y):
    return (int(math.floor(x / cell + 0.5)), int(math.floor(y / cell + 0.5)))


def key(c):
    return f"{c[0]},{c[1]}"


def unkey(k):
    a, b = k.split(",")
    return (int(a), int(b))


def score(grid, c):
    return grid.get(key(c), 0)


def bump(grid, c, by):
    grid[key(c)] = max(-TOP, min(TOP, grid.get(key(c), 0) + by))


def is_free(grid, c):
    return score(grid, c) <= FREE


def is_occ(grid, c):
    return score(grid, c) >= OCC


def add_readings(grid, cell, readings, beam_deg, reach_m, arcs=True):
    """Put a set of readings from one stretch (one scan, one run) in the map.

    readings: (x, y, bearing in degrees, distance in m or None for "nothing").
    A cell that any cone saw through, twice, is free. A cell that was on an
    arc and that no cone saw through is taken. Without a turn nothing clears
    the arcs, so give arcs=False for readings that all look one way: then
    they only mark free cells. Returns (free, taken) counts."""
    seen_free, on_arc = {}, {}
    half = beam_deg / 2
    for x, y, bearing, dist in readings:
        hit = dist is not None and dist <= reach_m
        d = dist if hit else reach_m
        n = int(d / cell) + 2
        cx0, cy0 = cell_of(cell, x, y)
        for ix in range(cx0 - n, cx0 + n + 1):
            for iy in range(cy0 - n, cy0 + n + 1):
                dx, dy = ix * cell - x, iy * cell - y
                r = math.hypot(dx, dy)
                if r > d + cell * 0.75:
                    continue
                if r > cell and abs(norm(math.degrees(math.atan2(dy, dx)) - bearing)) > half:
                    continue
                if r < d - cell * 0.75:
                    seen_free[(ix, iy)] = seen_free.get((ix, iy), 0) + 1
                elif hit:
                    on_arc[(ix, iy)] = on_arc.get((ix, iy), 0) + 1
    free = taken = 0
    for c, n in seen_free.items():
        if n >= 2 or c not in on_arc:
            bump(grid, c, -1 if c in on_arc else -2)
            free += 1
    for c in on_arc if arcs else ():
        if seen_free.get(c, 0) < 2:
            bump(grid, c, 2)
            taken += 1
    return free, taken


def wall_bearings(readings, beam_deg, flank=22.0, most=0.04):
    """The directions in a full scan that look straight at a wall, in degrees.

    The sensor gives the nearest thing in its cone. So while any part of the
    cone looks straight at a flat wall, the reading is the distance to the
    wall, D: a flat stretch as wide as the cone. Past it, the nearest point is
    at the edge of the cone, and the reading grows as D / cos(angle past the
    edge). This looks for that shape in the scan. The middle of the flat
    stretch is the direction of the wall.

    readings: (bearing in degrees, distance in m or None), in the order of the
    turn. Returns [(bearing, D, miss)], the bearings in 0 .. 360, the best
    first. `flank`: degrees past the flat stretch that are compared, on each
    side. `most`: the readings may be this far from the shape, as a share of
    D (the middle one of all the differences, so a few bad ones do no harm)."""
    # The scan on a grid of 1 degree.
    prof, last = [None] * 360, None
    for b, d in readings:
        if d is None:
            last = None
            continue
        if last and 0 < b - last[0] < 8:
            for deg in range(int(math.ceil(last[0])), int(math.floor(b)) + 1):
                t = (deg - last[0]) / (b - last[0])
                prof[deg % 360] = last[1] + (d - last[1]) * t
        last = (b, d)
    half = beam_deg / 2
    w = int(half + flank)

    def miss(c):
        """How far the scan around bearing c is from the shape of a wall, and D."""
        flat = [prof[int(math.floor(c + k + 0.5)) % 360] for k in range(-int(half) + 3, int(half) - 2)]
        flat = sorted(v for v in flat if v is not None)
        if len(flat) < half:
            return None
        dist = flat[len(flat) // 2]
        diffs, sides, rise = [], [0, 0], []
        for k in range(-w, w + 1):
            v = prof[int(math.floor(c + k + 0.5)) % 360]
            if v is None:
                continue
            past = abs(k) - half
            want = dist if past <= 0 else dist / math.cos(math.radians(past))
            diffs.append(abs(v - want) / dist)
            if past > 3:
                sides[k > 0] += 1
                rise.append(abs(v - want) / dist)
        # Both sides must be there: one side alone cannot say where the middle is.
        if min(sides) < flank / 2 or len(diffs) < 1.6 * w:
            return None
        # The flat stretch alone is no proof: the growth at both sides must fit too.
        diffs.sort()
        rise.sort()
        return max(diffs[len(diffs) // 2], rise[len(rise) // 2]), dist

    score = [miss(c) for c in range(360)]
    out = []
    for c in range(360):
        if score[c] is None or score[c][0] > most:
            continue
        near = [score[(c + k) % 360] for k in range(-10, 11)]
        if any(v is not None and v[0] < score[c][0] for v in near):
            continue  # a better one is next to it
        # A finer look around the best whole degree.
        best = min(((miss(c + f / 4.0) or (9,))[0], c + f / 4.0) for f in range(-4, 5))
        out.append((round(best[1] % 360, 2), round(score[c][1], 3), round(best[0], 4)))
    out.sort(key=lambda x: x[2])
    return out


def add_robot(grid, cell, x, y, radius):
    """Where the robot stands, nothing else is."""
    n = int(radius / cell) + 1
    cx, cy = cell_of(cell, x, y)
    for ix in range(cx - n, cx + n + 1):
        for iy in range(cy - n, cy + n + 1):
            if math.hypot(ix * cell - x, iy * cell - y) <= radius:
                bump(grid, (ix, iy), -TOP)


def add_bump(grid, cell, x, y, bearing, ahead):
    """The robot hit something it did not see: mark the cell `ahead` m in front."""
    th = math.radians(bearing)
    c = cell_of(cell, x + ahead * math.cos(th), y + ahead * math.sin(th))
    grid[key(c)] = TOP
    return c


def blocked_cells(grid, cell, keep_out):
    """Cells too near something for the robot's middle to be there."""
    out = set()
    n = int(keep_out / cell) + 1
    for k, v in grid.items():
        if v >= OCC:
            cx, cy = unkey(k)
            for ix in range(cx - n, cx + n + 1):
                for iy in range(cy - n, cy + n + 1):
                    if math.hypot(ix - cx, iy - cy) * cell <= keep_out:
                        out.add((ix, iy))
    return out


STEPS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]


def reach(grid, start, blocked):
    """Cost in cells from `start` to every free cell the robot can get to."""
    cost, heap = {start: 0.0}, [(0.0, start)]
    while heap:
        d, c = heapq.heappop(heap)
        if d > cost.get(c, 1e9):
            continue
        for dx, dy, w in STEPS:
            nb = (c[0] + dx, c[1] + dy)
            if nb in blocked or not is_free(grid, nb):
                continue
            if dx and dy and ((c[0] + dx, c[1]) in blocked or (c[0], c[1] + dy) in blocked):
                continue  # no cutting a corner
            nd = d + w
            if nd < cost.get(nb, 1e9):
                cost[nb] = nd
                heapq.heappush(heap, (nd, nb))
    return cost


def clear_line(grid, a, b, blocked):
    """True if the straight line between two cells only crosses free cells."""
    n = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2
    for i in range(n + 1):
        t = i / n if n else 0
        c = (int(math.floor(a[0] + (b[0] - a[0]) * t + 0.5)),
             int(math.floor(a[1] + (b[1] - a[1]) * t + 0.5)))
        if c != a and (c in blocked or not is_free(grid, c)):
            return False
    return True


def route(grid, start, goal, blocked):
    """Cells to go through from start to goal: only the corners. None if no way."""
    cost = reach(grid, start, blocked)
    if goal not in cost:
        return None
    path, c = [goal], goal
    while c != start:
        c = min(((c[0] + dx, c[1] + dy) for dx, dy, _ in STEPS
                 if (c[0] + dx, c[1] + dy) in cost), key=lambda q: cost[q])
        path.append(c)
    path.reverse()
    out, i = [], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not clear_line(grid, path[i], path[j], blocked):
            j -= 1
        out.append(path[j])
        i = j
    return out


def frontiers(grid, cost, least):
    """Groups of free cells the robot can get to that touch cells nobody has seen.
    Returns [(cells, nearest cell, its cost)], the nearest group first."""
    edge = set()
    for c in cost:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            v = score(grid, (c[0] + dx, c[1] + dy))
            if FREE < v < OCC:
                edge.add(c)
                break
    groups = []
    while edge:
        todo, group = [edge.pop()], []
        while todo:
            c = todo.pop()
            group.append(c)
            for dx, dy, _ in STEPS:
                nb = (c[0] + dx, c[1] + dy)
                if nb in edge:
                    edge.remove(nb)
                    todo.append(nb)
        if len(group) >= least:
            best = min(group, key=lambda q: cost[q])
            groups.append((group, best, cost[best]))
    groups.sort(key=lambda g: g[2])
    return groups


def draw(grid, cell, robot=None, heading=0.0, path=(), limit=(100, 60)):
    """The map as lines of text: # taken, . free, blank not seen. Forward at the
    start (x) is up, left (y) is to the left."""
    cells = {unkey(k): v for k, v in grid.items() if v >= OCC or v <= FREE}
    marks = dict(cells)
    for q in path:
        marks.setdefault(cell_of(cell, q[0], q[1]), FREE)
    rc = cell_of(cell, robot[0], robot[1]) if robot else None
    if rc:
        marks.setdefault(rc, FREE)
    marks.setdefault((0, 0), FREE)
    xs, ys = [c[0] for c in marks], [c[1] for c in marks]
    x1, x2, y1, y2 = min(xs), max(xs), min(ys), max(ys)
    if x2 - x1 >= limit[0] or y2 - y1 >= limit[1]:
        return None
    walked = {cell_of(cell, q[0], q[1]) for q in path}
    rows = []
    for ix in range(x2, x1 - 1, -1):
        row = []
        for iy in range(y2, y1 - 1, -1):
            c = (ix, iy)
            v = cells.get(c)
            ch = "#" if v is not None and v >= OCC else ("." if v is not None else " ")
            if c == (0, 0):
                ch = "o"
            if c in walked and ch == ".":
                ch = ":"
            if c == rc:
                ch = "^<v>"[int(((heading + 45) % 360) // 90)]
            row.append(ch)
        rows.append("".join(row).rstrip())
    return rows


def turn_period(headings, values, lo=320.0, hi=400.0):
    """How many reported degrees one real turn takes. The robot turns more than
    once on the spot and reads the distance all the way. The room is the same
    after one real turn, so the distances repeat. Returns (period, how sure):
    how sure is the mean error at other periods over the error at the best one."""
    if len(headings) < 20:
        return None, 0.0
    h0, h1 = min(headings), max(headings)
    pts = sorted(zip(headings, values))

    def at(h):
        lo_i, hi_i = 0, len(pts) - 1
        while hi_i - lo_i > 1:
            mid = (lo_i + hi_i) // 2
            if pts[mid][0] <= h:
                lo_i = mid
            else:
                hi_i = mid
        (ha, va), (hb, vb) = pts[lo_i], pts[hi_i]
        return va if hb == ha else va + (vb - va) * (h - ha) / (hb - ha)

    errs = []
    t = lo
    while t <= hi:
        span = h1 - h0 - t
        if span < 90:
            break
        n = int(span / 2)
        e = sum(abs(at(h0 + 2 * i) - at(h0 + 2 * i + t)) for i in range(n)) / n
        errs.append((e, t))
        t += 0.25
    if len(errs) < 10:
        return None, 0.0
    best = min(errs)
    mean = sum(e for e, _ in errs) / len(errs)
    return best[1], round(mean / max(best[0], 1e-6), 1)
