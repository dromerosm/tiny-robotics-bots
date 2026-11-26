// The page: reads /api/days, /api/hours and /api/prices and draws them.
// No libraries. Text from the API goes in with textContent, never innerHTML.

const TZ = "Europe/Madrid"; // the bridge's CONFIG_HOME_TZ
const SVG = "http://www.w3.org/2000/svg";
const $ = (id) => document.getElementById(id);
const kwh = (wh) => (wh / 1000).toLocaleString("en-GB", { minimumFractionDigits: 2, maximumFractionDigits: 2 });
const eur = (v) =>
  v.toLocaleString("en-GB", { style: "currency", currency: "EUR", minimumFractionDigits: 2, maximumFractionDigits: v < 1 ? 4 : 2 });
const axisText = (wh) => (wh < 1000 ? `${Math.round(wh)} Wh` : `${+(wh / 1000).toFixed(2)} kWh`);
const whText = (wh) => (wh < 1000 ? `${Math.round(wh)} Wh` : `${kwh(wh)} kWh`);

// Local day and hour keys in the bridge's time zone, the same form the API uses.
const parts = new Intl.DateTimeFormat("en-CA", {
  timeZone: TZ, year: "numeric", month: "2-digit", day: "2-digit", hour: "2-digit", hourCycle: "h23",
});
function keyOf(date) {
  const p = Object.fromEntries(parts.formatToParts(date).map((x) => [x.type, x.value]));
  return { day: `${p.year}-${p.month}-${p.day}`, hour: `${p.year}-${p.month}-${p.day}T${p.hour}` };
}
function lastDays(n) {
  const out = [];
  for (let i = n - 1; i >= 0; i--) out.push(keyOf(new Date(Date.now() - i * 86400000)).day);
  return [...new Set(out)];
}
function lastHours(n) {
  const out = [];
  for (let i = n - 1; i >= 0; i--) out.push(keyOf(new Date(Date.now() - i * 3600000)).hour);
  return [...new Set(out)]; // the autumn clock change repeats an hour: one slot
}

async function get(path) {
  const r = await fetch(path, { headers: { Accept: "application/json" } });
  if (!r.ok) throw new Error(`${path}: ${r.status}`);
  return r.json();
}

// One color per device, by its id, so a range with fewer devices does not repaint the rest.
let colorOf = new Map();
function setColors(rows) {
  const ids = [...new Map(rows.map((r) => [r.dev, r.name]))].sort((a, b) => a[0].localeCompare(b[0]));
  ids.forEach(([id], i) => colorOf.has(id) || colorOf.set(id, `var(--s${(colorOf.size % 8) + 1})`));
  return ids;
}

function el(name, attrs = {}, parent) {
  const e = document.createElementNS(SVG, name);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  if (parent) parent.appendChild(e);
  return e;
}

function niceMax(v) {
  if (v <= 0) return 1;
  const p = 10 ** Math.floor(Math.log10(v));
  return [1, 2, 2.5, 5, 10].map((m) => m * p).find((m) => m >= v);
}

// Stacked bars: keys along x, one segment per device, in Wh.
function bars(box, keys, rows, keyField, devs, label) {
  box.replaceChildren();
  const byKey = new Map(keys.map((k) => [k, new Map()]));
  for (const r of rows) byKey.get(r[keyField])?.set(r.dev, r);
  const totals = keys.map((k) => [...byKey.get(k).values()].reduce((s, r) => s + r.wh, 0));
  if (!totals.some((t) => t > 0)) {
    const p = document.createElement("p");
    p.className = "empty";
    p.textContent = "No data for this range yet.";
    box.appendChild(p);
    return;
  }
  const W = 900, H = 240, L = 64, R = 8, T = 8, B = 24;
  const max = niceMax(Math.max(...totals));
  const y = (v) => T + (H - T - B) * (1 - v / max);
  const step = (W - L - R) / keys.length;
  const bw = Math.max(1, Math.min(step - 2, 28));
  const svg = el("svg", { viewBox: `0 0 ${W} ${H}`, role: "img", "aria-label": label }, box);
  for (let i = 0; i <= 4; i++) {
    const v = (max / 4) * i, yy = y(v);
    el("line", { x1: L, x2: W - R, y1: yy, y2: yy, class: i ? "grid" : "base" }, svg);
    el("text", { x: L - 6, y: yy + 4, "text-anchor": "end" }, svg).textContent = axisText(v);
  }
  const every = Math.ceil(keys.length / 10);
  keys.forEach((k, i) => {
    const x = L + step * i + (step - bw) / 2;
    let acc = 0;
    const segs = devs.map(([id]) => byKey.get(k).get(id)).filter((r) => r && r.wh > 0);
    segs.forEach((r, j) => {
      const top = y(acc + r.wh), h = y(acc) - top;
      acc += r.wh;
      // Round only the top end of the bar, anchored at the baseline.
      const last = j === segs.length - 1, rr = last ? Math.min(4, bw / 2, h) : 0;
      el("path", {
        d: `M${x},${top + h}V${top + rr}q0,${-rr} ${rr},${-rr}H${x + bw - rr}q${rr},0 ${rr},${rr}V${top + h}Z`,
        fill: colorOf.get(r.dev), class: "seg",
      }, svg);
    });
    if (i % every === 0) {
      el("text", { x: x + bw / 2, y: H - 6, "text-anchor": "middle" }, svg).textContent =
        k.includes("T") ? `${k.slice(11)}h` : k.slice(5);
    }
    const hit = el("rect", { x: L + step * i, y: T, width: step, height: H - T - B, class: "hit" }, svg);
    hit.addEventListener("mousemove", (e) => tip(e, k, segs, totals[i]));
    hit.addEventListener("mouseleave", () => ($("tip").hidden = true));
  });
}

function tip(e, key, segs, total) {
  const t = $("tip");
  t.replaceChildren();
  const b = document.createElement("b");
  b.textContent = key.includes("T") ? `${key.slice(0, 10)} ${key.slice(11)}:00` : key;
  t.appendChild(b);
  for (const r of [...segs].reverse()) {
    const row = document.createElement("div");
    row.className = "row";
    const name = document.createElement("span");
    const sw = document.createElement("span");
    sw.className = "swatch";
    sw.style.setProperty("--c", colorOf.get(r.dev));
    name.append(sw, r.name);
    const v = document.createElement("span");
    v.textContent = `${whText(r.wh)} · ${eur(r.eur)}`;
    row.append(name, v);
    t.appendChild(row);
  }
  const sum = document.createElement("div");
  sum.className = "row";
  sum.textContent = `Total: ${whText(total)}`;
  t.appendChild(sum);
  t.hidden = false;
  const x = Math.min(e.clientX + 12, innerWidth - t.offsetWidth - 8);
  t.style.left = `${x}px`;
  t.style.top = `${e.clientY + 12}px`;
}

function cell(tr, text, cls, tag = "td") {
  const c = document.createElement(tag);
  if (cls) c.className = cls;
  if (text instanceof Node) c.appendChild(text); else c.textContent = text;
  tr.appendChild(c);
  return c;
}

function totals(rows, devs) {
  const t = $("totals");
  t.replaceChildren();
  const head = t.createTHead().insertRow();
  ["Device", "kWh", "Cost"].forEach((h, i) => cell(head, h, i ? "n" : "", "th"));
  const body = t.createTBody();
  let all = { wh: 0, eur: 0 };
  for (const [id, name] of devs) {
    const s = rows.filter((r) => r.dev === id).reduce((a, r) => ({ wh: a.wh + r.wh, eur: a.eur + r.eur }), { wh: 0, eur: 0 });
    all = { wh: all.wh + s.wh, eur: all.eur + s.eur };
    const tr = body.insertRow();
    const n = document.createElement("span");
    const sw = document.createElement("span");
    sw.className = "swatch";
    sw.style.setProperty("--c", colorOf.get(id));
    n.append(sw, name);
    cell(tr, n);
    cell(tr, kwh(s.wh), "n");
    cell(tr, eur(s.eur), "n");
  }
  const foot = t.createTFoot().insertRow();
  cell(foot, "All");
  cell(foot, kwh(all.wh), "n");
  cell(foot, eur(all.eur), "n");
  return all;
}

function tiles(all, n, today) {
  const box = $("tiles");
  box.replaceChildren();
  const add = (label, value) => {
    const d = document.createElement("div");
    d.className = "tile";
    const l = document.createElement("div"); l.className = "label"; l.textContent = label;
    const v = document.createElement("div"); v.className = "value"; v.textContent = value;
    d.append(l, v);
    box.appendChild(d);
  };
  add(`Energy, last ${n} days`, `${kwh(all.wh)} kWh`);
  add(`Cost, last ${n} days`, eur(all.eur));
  add("Today so far", `${kwh(today.wh)} kWh · ${eur(today.eur)}`);
}

async function prices() {
  const t = $("prices");
  t.replaceChildren();
  const head = t.createTHead().insertRow();
  ["From", "EUR/kWh", "Note"].forEach((h, i) => cell(head, h, i === 1 ? "n" : "", "th"));
  const body = t.createTBody();
  for (const p of await get("/api/prices")) {
    const tr = body.insertRow();
    cell(tr, p.valid_from);
    cell(tr, p.eur_kwh.toFixed(6), "n");
    cell(tr, p.note || "");
  }
}

async function load(n) {
  $("status").textContent = "Loading…";
  try {
    const days = lastDays(n), hours = lastHours(48);
    const [d, h] = await Promise.all([
      get(`/api/days?from=${days[0]}&to=${days.at(-1)}`),
      get(`/api/hours?from=${hours[0].slice(0, 10)}&to=${hours.at(-1).slice(0, 10)}`),
    ]);
    const devs = setColors([...d, ...h]);
    const legend = $("legend");
    legend.replaceChildren();
    for (const [id, name] of devs) {
      const li = document.createElement("li");
      li.style.setProperty("--c", colorOf.get(id));
      li.textContent = name;
      legend.appendChild(li);
    }
    bars($("days-chart"), days, d, "d", devs, `Energy per day and device, last ${n} days`);
    bars($("hours-chart"), hours, h, "t", devs, "Energy per hour and device, last 48 hours");
    const all = totals(d, devs);
    const today = d.filter((r) => r.d === days.at(-1)).reduce((a, r) => ({ wh: a.wh + r.wh, eur: a.eur + r.eur }), { wh: 0, eur: 0 });
    tiles(all, n, today);
    $("status").textContent = `Updated ${new Date().toLocaleTimeString("en-GB", { timeZone: TZ })} (${TZ}).`;
  } catch (e) {
    $("status").textContent = `Could not load the data: ${e.message}`;
  }
}

document.querySelectorAll(".filters button").forEach((b) =>
  b.addEventListener("click", () => {
    document.querySelectorAll(".filters button").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
    load(Number(b.dataset.days));
  }),
);
load(30);
prices();
