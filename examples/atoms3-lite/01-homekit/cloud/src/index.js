// home-energy: keeps the energy the HomeKit bridge measures, with no end date.
//
// POST /ingest  The bridge sends its /api/energy JSON (the last 48 hours and
//               62 days) every hour, with "Authorization: Bearer <INGEST_TOKEN>".
//               Every slot is an upsert that keeps the larger value, so a lost
//               send is covered by the next one, and a bridge that lost its
//               history cannot lower what is stored.
// GET /api/hours?from=2026-11-01&to=2026-11-30   Wh and EUR per device and hour.
// GET /api/days?from=2026-01-01&to=2026-12-31    Wh and EUR per device and day.
// GET /api/prices                                The price table.
// Everything else is the page in public/. Cloudflare Access guards it all but
// /ingest.

const DAY = /^\d{4}-\d{2}-\d{2}$/;
const HOUR = /^\d{4}-\d{2}-\d{2}T\d{2}$/;

// The price that holds on the slot's day.
const PRICE = `(SELECT eur_kwh FROM prices p WHERE p.valid_from <= substr(x.k, 1, 10)
                ORDER BY p.valid_from DESC LIMIT 1)`;

export default {
  async fetch(req, env) {
    const url = new URL(req.url);
    try {
      if (url.pathname === "/ingest") {
        return req.method === "POST" ? await ingest(req, env) : text("POST only", 405);
      }
      if (req.method !== "GET") {
        return text("GET only", 405);
      }
      switch (url.pathname) {
        case "/api/hours":
          return await series(env, url, "hours", "t");
        case "/api/days":
          return await series(env, url, "days", "d");
        case "/api/prices":
          return json((await env.DB.prepare("SELECT * FROM prices ORDER BY valid_from").all()).results);
      }
      return env.ASSETS.fetch(req);
    } catch (e) {
      if (e instanceof Bad) {
        return text(e.message, 400);
      }
      console.error(e);
      return text("error", 500);
    }
  },
};

class Bad extends Error {}

async function ingest(req, env) {
  if (!env.INGEST_TOKEN || !same(req.headers.get("Authorization") || "", `Bearer ${env.INGEST_TOKEN}`)) {
    return text("wrong or missing token", 403);
  }
  if (Number(req.headers.get("Content-Length") || 0) > 65536) {
    return text("too big", 413);
  }
  let body;
  try {
    body = await req.json();
  } catch {
    throw new Bad("not JSON");
  }
  const devs = Array.isArray(body?.devices) ? body.devices : [];
  for (const d of devs) {
    if (typeof d?.id !== "string" || !d.id || d.id.length > 32 || typeof d.name !== "string") {
      throw new Bad("each device needs an id and a name");
    }
  }
  const stmts = devs.map((d) =>
    env.DB.prepare(
      "INSERT INTO devices (id, name) VALUES (?, ?) ON CONFLICT (id) DO UPDATE SET name = excluded.name",
    ).bind(d.id, d.name.slice(0, 64)),
  );
  const slots = (list, key, re, table, col) => {
    for (const s of Array.isArray(list) ? list : []) {
      if (typeof s?.[key] !== "string" || !re.test(s[key]) || !Array.isArray(s.wh)) {
        throw new Bad(`bad ${table} slot`);
      }
      devs.forEach((d, i) => {
        const wh = Number(s.wh[i]);
        if (Number.isFinite(wh) && wh >= 0) {
          stmts.push(
            env.DB.prepare(
              `INSERT INTO ${table} (dev, ${col}, wh) VALUES (?, ?, ?)
               ON CONFLICT (dev, ${col}) DO UPDATE SET wh = max(wh, excluded.wh)`,
            ).bind(d.id, s[key], wh),
          );
        }
      });
    }
  };
  slots(body.hours, "t", HOUR, "hours", "t");
  slots(body.days, "d", DAY, "days", "d");
  if (stmts.length) {
    await env.DB.batch(stmts);
  }
  return json({ ok: true, rows: stmts.length - devs.length });
}

async function series(env, url, table, col) {
  const from = url.searchParams.get("from") || "0000-00-00";
  const to = url.searchParams.get("to") || "9999-99-99";
  if (!DAY.test(from) || !DAY.test(to)) {
    throw new Bad("from and to are days: 2026-11-26");
  }
  // "to" is a whole day: an hour "2026-11-26T23" is still in it.
  const { results } = await env.DB.prepare(
    `SELECT x.k AS ${col}, x.dev, d.name, x.wh, round(x.wh / 1000 * ${PRICE}, 4) AS eur
     FROM (SELECT ${col} AS k, dev, wh FROM ${table}) x JOIN devices d ON d.id = x.dev
     WHERE substr(x.k, 1, 10) BETWEEN ? AND ? ORDER BY x.k, d.name`,
  )
    .bind(from, to)
    .all();
  return json(results);
}

// Compare two strings in a time that does not depend on where they differ.
function same(a, b) {
  const x = new TextEncoder().encode(a);
  const y = new TextEncoder().encode(b);
  let diff = x.length ^ y.length;
  for (let i = 0; i < Math.max(x.length, y.length); i++) {
    diff |= (x[i] || 0) ^ (y[i] || 0);
  }
  return diff === 0;
}

function json(data, status = 200) {
  return Response.json(data, { status, headers: { "Cache-Control": "no-store" } });
}

function text(msg, status) {
  return new Response(msg, { status, headers: { "Content-Type": "text/plain; charset=utf-8" } });
}
