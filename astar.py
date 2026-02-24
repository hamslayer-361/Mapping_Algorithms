import heapq, math, argparse

# grid file:
# S start, G goal, . free, # wall


def read_map(fn):
    lines = []
    with open(fn, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip("\n")
            if line.strip() == "":
                continue
            lines.append([ch for ch in line if ch != " "])

    g = []
    s = None
    t = None
    for r in range(len(lines)):
        row = []
        for c in range(len(lines[r])):
            ch = lines[r][c]
            if ch == "S":
                s = (r, c)
                row.append(0)
            elif ch == "G":
                t = (r, c)
                row.append(0)
            elif ch == ".":
                row.append(0)
            elif ch == "#":
                row.append(1)
            else:
                raise ValueError("bad char: " + ch)
        g.append(row)

    if s is None or t is None:
        raise ValueError("need S and G")

    w = len(g[0])
    for rr in g:
        if len(rr) != w:
            raise ValueError("rows not same length")
    return g, s, t


def h_oct(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    m = min(dx, dy)
    M = max(dx, dy)
    return (M - m) + m * math.sqrt(2)

def h_man(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def h_zero(a, b):
    return 0

def show(g, path, s, t):
    ps = set(path)
    out = []
    for r in range(len(g)):
        line = []
        for c in range(len(g[0])):
            p = (r, c)
            if p == s:
                line.append("S")
            elif p == t:
                line.append("G")
            elif g[r][c] == 1:
                line.append("#")
            elif p in ps:
                line.append("·")
            else:
                line.append(".")
        out.append("".join(line))
    return "\n".join(out)


def astar(g, s, t, hh):
    R = len(g)
    C = len(g[0])

    def ok(p):
        return 0 <= p[0] < R and 0 <= p[1] < C and g[p[0]][p[1]] == 0

    # 8 dirs
    diag = math.sqrt(2)
    moves = [(-1,0,1), (1,0,1), (0,-1,1), (0,1,1),
             (-1,-1,diag), (-1,1,diag), (1,-1,diag), (1,1,diag)]

    # these are kinda messy but work
    came = {s: None}
    dist = {s: 0.0}

    q = []
    heapq.heappush(q, (hh(s, t), 0, s))
    seen = set()
    pops = 0
    tie = 0

    while q:
        _, _, cur = heapq.heappop(q)
        if cur in seen:
            continue
        seen.add(cur)
        pops += 1

        if cur == t:
            break

        for dr, dc, cost in moves:
            nx = (cur[0] + dr, cur[1] + dc)

            if not ok(nx):
                continue

            # no cutting corners (if diagonal, both sides must be free)
            if dr != 0 and dc != 0:
                a = (cur[0] + dr, cur[1])
                b = (cur[0], cur[1] + dc)
                if not (ok(a) and ok(b)):
                    continue

            nd = dist[cur] + cost
            if nd < dist.get(nx, 10**18):
                dist[nx] = nd
                came[nx] = cur
                tie += 1
                heapq.heappush(q, (nd + hh(nx, t), tie, nx))

    # rebuild path
    if t not in came:
        return [], math.inf, pops

    p = []
    cur = t
    while cur is not None:
        p.append(cur)
        cur = came.get(cur)
    p.reverse()
    return p, dist.get(t, math.inf), pops


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("PythonProject\mapfile")
    ap.add_argument("--algo", choices=["astar", "dijkstra"], default="astar")
    ap.add_argument("--heur", choices=["octile", "manhattan"], default="octile")
    args = ap.parse_args()

    g, s, t = read_map(args.mapfile)

    if args.algo == "dijkstra":
        hh = h_zero
        name = "dijkstra"
    else:
        hh = h_oct if args.heur == "octile" else h_man
        name = "astar-" + args.heur

    path, cost, popped = astar(g, s, t, hh)

    print("algo:", name)
    print("visited:", popped)
    if not path:
        print("no path")
    else:
        print("cost:", round(cost, 3))
        print("steps:", len(path) - 1)
        print(show(g, path, s, t))